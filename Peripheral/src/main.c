#include <stdio.h> // Inclui o cabecalho padrao de entrada e saida
#include <stddef.h> // Inclui o cabecalho para tipos e macros relacionados a manipulacao de ponteiros
#include <string.h> // Inclui o cabecalho para funcoes relacionadas a manipulacao de strings.
#include <errno.h> // tratamento de erros
#include <zephyr/types.h> // Inclui tipos de dados especificos do Zephyr
#include <sys/printk.h> // Inclui funcoes de impressao para depuracao
#include <sys/byteorder.h> // Inclui funcoes para lidar com a ordem dos bytes (endianness)
#include <zephyr.h> // Inclui o cabecalho principal do Zephyr
#include <kernel.h> // Inclui o cabecalho do kernel do Zephyr
#include <services/application.h> // servico de aplicacao do Zephyr
#include <services/peripheral.h> // servico periferico do Zephyr

/**
 * Esta função é um callback que será chamado quando dados forem recebidos via BLE (Bluetooth Low Energy).
 * Ela recebe um ponteiro para um buffer de dados (buffer) e o tamanho desse buffer (len).
 */
static void on_ble_rx_data(const uint8_t *buffer, size_t len) {
    ble_uart_service_transmit(buffer, len);
}

/**
 * Esta função é um callback chamado quando a pilha BLE está pronta para uso.
 *
 * @ param conn Um ponteiro para uma estrutura bt_conn (conexão Bluetooth). Não utilizado neste caso.
 */
static void on_ble_stack_ready(struct bt_conn *conn) {
    (void)conn;
    /**
     * Isso é uma declaração que está sendo usada para evitar um aviso (warning) do compilador 
     * relacionado ao parâmetro conn que não está sendo utilizado na função on_ble_stack_ready. 
    */
    ble_uart_service_register(on_ble_rx_data);
    /**
     * Chama a função ble_uart_service_register com on_ble_rx_data como argumento. Essa função 
     * registrando on_ble_rx_data como um callback para receber dados da pilha Bluetooth Low Energy (BLE).
     * Portanto, quando dados BLE são recebidos e prontos para serem manipulados, a função on_ble_rx_data 
     * será chamada para processar esses dados.
    */
}

/**
 * configura o ponto de entrada do programa e inicia a aplicação BLE, registrando os callbacks 
 * necessários para lidar com a comunicação BLE e os dados recebidos.
*/
void main (void) {
    /**
     * essa função recebe dados no buffer buffer, converte caracteres minúsculos em maiúsculos (se aplicável)
     * e depois notifica a conexão BLE com os dados processados, usando a função bt_gatt_notify.
    */
    ble_application_start(on_ble_stack_ready);
}