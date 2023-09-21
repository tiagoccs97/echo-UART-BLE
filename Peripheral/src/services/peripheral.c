#include <services/peripheral.h>

// Declaração de um ponteiro de função de callback para o serviço BLE
static ble_uart_service_rx_callback rx_callback = NULL;

// Definição de um deslocamento para o caractere de transmissão no serviço BLE
#define BLE_UART_SERVICE_TX_CHAR_OFFSET    3

// Tamanho máximo do buffer de dados da característica BLE
#define CHRC_SIZE 100
static uint8_t chrc_data[CHRC_SIZE];

// Macro para criar uma flag booleana atômica chamada 'flag'
#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t)false

// Macro para definir uma flag booleana como verdadeira
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)true)

// Macro para definir uma flag booleana como falsa
#define UNSET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)false)

// Macro para esperar até que uma flag booleana se torne verdadeira
#define WAIT_FOR_FLAG(flag) \
	while (!(bool)atomic_get(&flag)) { \
		(void)k_sleep(K_MSEC(1)); \
	}

// Criação de uma flag booleana atômica chamada 'flag_long_subscribe'
CREATE_FLAG(flag_long_subscribe);

// Definição de uma UUID (Identificador Único Universal) para o serviço BLE 'ble_uart_uppercase'
static struct bt_uuid_128 ble_uart_uppercase = BT_UUID_INIT_128(
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00
);

// Definição de uma UUID para a característica BLE 'ble_uart_receive_data'
static struct bt_uuid_128 ble_uart_receive_data = BT_UUID_INIT_128(
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00
);

// Definição de uma UUID para a característica BLE 'ble_uart_notify'
static struct bt_uuid_128 ble_uart_notify = BT_UUID_INIT_128(
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x11
);

// Função para lidar com a escrita de dados em uma característica BLE
static ssize_t write_test_chrc(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{
    // Variável para contar operações de preparação (preparação para escrita)
    static uint8_t prepare_count;

    // Converte os dados de entrada em uma string null-terminated para fins de impressão
    uint8_t string[len + 1];
    for (int i = 0; i < len; i++) {
        string[i] = *((char *)buf + i);
    }
    string[len] = '\0';

    // Imprime os dados recebidos para fins de depuração
    printk("\nReceived data: %s\n", string);

    // Verifica se o comprimento dos dados excede o tamanho máximo permitido
    if (len > sizeof(chrc_data)) {
        printk("Invalid length\n");
        // Retorna um erro indicando comprimento inválido
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    } 
    // Verifica se o deslocamento e o comprimento excedem o tamanho máximo do buffer
    else if (offset + len > sizeof(chrc_data)) {
        printk("Invalid offset and length\n");
        // Retorna um erro indicando deslocamento e comprimento inválidos
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    // Verifica se a operação é uma preparação (preparação para escrita)
    if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
        // Incrementa e imprime o contador de operações de preparação
        printk("prepare_count %u\n", prepare_count++);
        // Retorna sucesso, indicando que a preparação foi concluída com êxito
        return BT_GATT_ERR(BT_ATT_ERR_SUCCESS);
    }

    // Copia os dados recebidos para o buffer da característica BLE
    (void)memcpy(chrc_data + offset, buf, len);
    // Reinicia o contador de operações de preparação
    prepare_count = 0;

    // Chama o callback de recebimento de dados, se estiver definido
    if (rx_callback) {
        // Chama o callback com os dados recebidos e seu comprimento
        rx_callback((const uint8_t *)buf, len);
    }

    // Limpa o buffer de entrada (Não está claro o que essa linha faz, pode ser removida)
    buf = "";

    // Retorna o comprimento dos dados escritos
    return len;
}

// Declaração do array de atributos GATT para configurar o serviço BLE
static struct bt_gatt_attr attrs[] = {
    // Declaração do serviço BLE como serviço primário
    BT_GATT_PRIMARY_SERVICE(&ble_uart_uppercase),

    // Declaração da característica BLE para recebimento de dados
    // - A característica permite escrita sem resposta
    // - Possui permissões de escrita
    // - A função write_test_chrc é chamada quando dados são escritos nela
    BT_GATT_CHARACTERISTIC(&ble_uart_receive_data.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_WRITE, NULL, write_test_chrc, NULL),

    // Declaração da característica BLE para notificações
    // - Não permite escrita
    // - Usada para notificar clientes quando novos dados estão disponíveis
    BT_GATT_CHARACTERISTIC(&ble_uart_notify.uuid, BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_NONE, NULL, NULL, NULL),

    // Configuração do Cliente Characteristic Configuration (CCC)
    // - Permite que os clientes ativem ou desativem notificações
    // - A função ble_uart_ccc_changed é chamada quando o CCC é modificado
    // - Possui permissões de escrita para permitir que os clientes controlem as notificações
    BT_GATT_CCC(ble_uart_ccc_changed, BT_GATT_PERM_WRITE),
};

/**
 * Cria uma estrutura de serviço GATT (Generic Attribute Profile) BLE personalizado
 * chamado 'peripheral' e a inicializa com os atributos definidos em 'attrs'.
 */
static struct bt_gatt_service peripheral = BT_GATT_SERVICE(attrs);

/**
 * Esta função é chamada quando a configuração do Cliente de Característica de Configuração (CCC)
 * para as notificações BLE é alterada.
 *
 * @ param attr Um ponteiro para o atributo GATT relacionado à mudança do CCC.
 * @ param value O novo valor do CCC, que pode ser BT_GATT_CCC_NOTIFY (notificações ativadas) ou
 *              BT_GATT_CCC_INDICATE (indicações ativadas).
 */
static void ble_uart_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value){
    // Verifica se as notificações estão habilitadas (BT_GATT_CCC_NOTIFY).
    const bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

    // Se as notificações foram habilitadas, define a bandeira 'flag_long_subscribe'.
	if (notif_enabled) {
		SET_FLAG(flag_long_subscribe);
	}

    // Imprime uma mensagem indicando se as notificações estão habilitadas ou desabilitadas.
	printk("Notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

/**
 * Registra um callback para receber dados emble_uart_service_register uma aplicação Bluetooth Low Energy (BLE) personalizada.
 *
 * @ param callback Um ponteiro para a função de callback que será chamada quando os dados forem recebidos.
 *
 * @ return 0 em caso de sucesso, ou um código de erro em caso de falha.
 */
int ble_uart_service_register(const ble_uart_service_rx_callback callback) {
    // Define o callback de recebimento como o fornecido como argumento.
    rx_callback = callback;

    // Registra o serviço BLE, geralmente usado para expor serviços e características BLE personalizadas.
    // O detalhe da implementação de bt_gatt_service_register não está incluído aqui.
	return 	bt_gatt_service_register(&peripheral);
}

/**
 * Esta função transmite dados via Bluetooth Low Energy (BLE).
 *
 * @ param buffer Um ponteiro para o buffer de dados a ser transmitido.
 * @ param len O tamanho dos dados a serem transmitidos.
 *
 * @ return 0 em caso de sucesso, -1 em caso de erro.
 */
int ble_uart_service_transmit(const uint8_t *buffer, size_t len) {
    // Imprime uma mensagem de início de transmissão no log.
    printk("\n-----Start Transmit!-----\n");

    // Verifica se o buffer ou o tamanho dos dados são inválidos.
	if(!buffer || !len) {
		return -1;
	}

    // Obtém uma referência para a conexão BLE.
    struct bt_conn *conn = ble_get_connection_ref();

    // Cria um novo buffer para armazenar os dados modificados (se necessário).
    uint8_t string[len+1];
    // Itera sobre os elementos do buffer e realiza modificações (conversão de minúsculas para maiúsculas, se aplicável).
    for(int i = 0; i < len;i++){
        if(buffer[i] <= 'z' && buffer[i] >= 'a'){
            string[i] = buffer[i]-('a'-'A');
        }else{
            string[i] = buffer[i];
        }
    }
    // Adiciona um caractere nulo no final do buffer 'string' para torná-lo uma string C válida.
    string[len] = '\0';
    //printk("\n%s\n",string);
    // Verifica se a conexão BLE é válida.
    if(conn) {
        // Notifica a conexão BLE com os dados processados.
        return ( bt_gatt_notify(conn,
                            &attrs[2],
                            string,
                            len));
    } else {
        // Retorna -1 em caso de erro (conexão BLE inválida).
        return -1;
    }
}