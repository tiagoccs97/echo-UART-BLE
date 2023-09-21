
#include <services/application.h>

/**
 * BT_DATA_FLAGS: Define as configurações de flags do anúncio BLE.
 * BT_DATA_UUID16_ALL: Define os serviços UUID que o dispositivo oferece.
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0d, 0x18, 0x0f, 0x18, 0x0a, 0x18),
};

/**
 * Ponteiro para a conexão Bluetooth ativa.
 */
static struct bt_conn *ble_connection = NULL;

/**
 * Callback de prontidão BLE.
 */
static ble_ready_callback ready_callback = NULL;

/**
 * Callbacks de eventos de conexão BLE.
 */
static struct bt_conn_cb conn_callbacks;

/**
 * Status da conexão BLE.
 */
static ble_status status = kBleDisconnected;

/**
 * Inicia a publicidade BLE.
 *
 * return Um valor de status da operação de início da publicidade BLE.
 */
static uint32_t ble_start_advertise(void)  {
    return (bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0));
}

/**
 * Callback para lidar com eventos de conexão BLE bem-sucedidos.
 *
 * param conn Um ponteiro para a estrutura 'bt_conn' representando a conexão BLE estabelecida.
 * param err  O código de erro (0 para sucesso).
 */
static void ble_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk(" BLE connection failed (err %u)\n", err);
    } else {
        // Referencia a conexão BLE estabelecida.
        ble_connection = bt_conn_ref(conn);
        status = kBleConnected;
        printk(" BLE connected! \n");
    }
}

/**
 * Callback para lidar com eventos de desconexão BLE.
 *
 * param conn Um ponteiro para a estrutura 'bt_conn' representando a conexão BLE.
 * param reason O motivo da desconexão.
 */
static void ble_disconnected(struct bt_conn *conn, uint8_t reason) {
    // Imprime uma mensagem indicando que a conexão BLE foi desconectada e a razão.
    printk(" BLE disconnected, reason: %u \n", reason);

    // Verifica se há uma conexão BLE ativa e a desreferencia (libera) se existir.
    if (ble_connection) {
        bt_conn_unref(ble_connection);
        ble_connection = NULL;
    }

    // Define o status como "Desconectado" e reinicia a publicidade BLE.
    status = kBleDisconnected;
    ble_start_advertise();

    // Imprime uma mensagem indicando que a BLE foi reiniciada para publicidade.
    printk(" ble re-started to advertise ! \n");
}

/**
 * Callback para lidar com a notificação de que a pilha BLE está pronta.
 *
 * param err O código de erro (não utilizado neste caso).
 */
static void ble_stack_ready(int err) {
    // Suprime o warning "unused parameter" para 'err'.
    (void)err;

    // Chama a função de callback de prontidão BLE, se estiver definida.
    if (ready_callback) {
        ready_callback(ble_connection);
    }

    // A pilha BLE está inicializada, portanto, inicia a publicidade BLE.
    ble_start_advertise();

    // Imprime uma mensagem indicando que a BLE foi iniciada para publicidade.
    printk(" ble started to advertise ! \n");
}

/**
 * Inicia a aplicação Bluetooth Low Energy (BLE) e registra callbacks de conexão.
 *
 * param callback Uma função de callback que será chamada quando o BLE estiver pronto para uso.
 *
 * return Um valor de status. 0 indica sucesso e -1 indica erro.
 */
uint32_t ble_application_start(ble_ready_callback callback) {
    // Verifica se o callback fornecido é válido (não é nulo)
    if (!callback)
        return (-1);

    // Registra as funções de callback de conexão para eventos de conexão e desconexão
    conn_callbacks.connected = ble_connected;
    conn_callbacks.disconnected = ble_disconnected;

    // Define a função de callback de pronto para uso
    ready_callback = callback;

    // Registra os callbacks de conexão para serem chamados em eventos de conexão
    bt_conn_cb_register(&conn_callbacks);

    // Inicia a pilha BLE e associa a função 'ble_stack_ready' ao evento de prontidão BLE
    return ((uint32_t)bt_enable(ble_stack_ready));
}

// Função para obter uma referência à conexão Bluetooth ativa
// Retorna um ponteiro para a estrutura 'bt_conn' que representa a conexão atual.
struct bt_conn *ble_get_connection_ref() {
	return ble_connection;
}