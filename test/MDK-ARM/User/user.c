#include "user.h"
uint8_t tcp_buf[TCP_BUF_SIZE];

float linear = 0.0;
float angular = 0.0;

float accel_set = 10.0;
float decel_set = 10.0;

/* Private functions ---------------------------------------------------------*/
/* USER CODE PRIVATE FUNCTIONS ' DEFINITIONS */
void RS485_1_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
		HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, 1); 
		delayMicros(100);
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
		while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != 1);
		delayMicros(10);
		HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, 0);
    va_end(args);
}
void RS485_2_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
		HAL_GPIO_WritePin(RS485_2_RW_GPIO_Port, RS485_2_RW_Pin, 1); 
		delayMicros(100);
    HAL_UART_Transmit(&huart3, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != 1);
		delayMicros(10);
		HAL_GPIO_WritePin(RS485_2_RW_GPIO_Port, RS485_2_RW_Pin, 0);
    va_end(args);
}
/**************************TCP_W5500*************************************/
void W5500_Reset(void) {
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
}
void W5500_Select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, 20);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, 20);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

void W5500_Init_StaticIP(void) {
    // 1. Ðang ký callback SPI và CS
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

    // 2. C?p phát b? d?m RX/TX cho các socket (2KB m?i socket)
    uint8_t tx_rx_buf[8] = {2,2,2,2,2,2,2,2};
    wizchip_init(tx_rx_buf, tx_rx_buf);

    // 3. Gán IP tinh cho W5500
    wiz_NetInfo net = {
        .mac = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03},
        .ip  = {192, 168, 2, 123},
        .sn  = {255, 255, 255, 0},
        .gw  = {192, 168, 1, 1},
        .dhcp = NETINFO_STATIC
    };
    wizchip_setnetinfo(&net);

    // 4. Ki?m tra VERSIONR d? xác nh?n SPI ho?t d?ng
    uint8_t version = getVERSIONR();
    if (version == 0x04) {
        RS485_2_Printf("W5500 ready at IP: %d.%d.%d.%d\r\n",
            net.ip[0], net.ip[1], net.ip[2], net.ip[3]);
    } else {
        RS485_2_Printf("W5500 VERSIONR error: 0x%02X\r\n", version);
    }
}
void TCP_Server_Init(void) {
    // Ð?m b?o socket dã dóng
    close(TCP_SOCKET);
    HAL_Delay(10);

    // M? socket TCP server
    int8_t res = socket(TCP_SOCKET, Sn_MR_TCP, TCP_PORT, 0);
    if (res != TCP_SOCKET) {
        RS485_2_Printf("Failed to open TCP socket (%d)\r\n", res);
        return;
    }

    RS485_2_Printf("TCP socket opened on port %d. Waiting for client...\r\n", TCP_PORT);

    // Chuy?n socket sang ch? d? l?ng nghe (listen)
    listen(TCP_SOCKET);
}
void TCP_Server_Poll(void) {
    sock_stat = getSn_SR(TCP_SOCKET);
    switch (sock_stat) {
        case SOCK_CLOSED:
            socket(TCP_SOCKET, Sn_MR_TCP, TCP_PORT, 0);
            break;

        case SOCK_INIT:
            listen(TCP_SOCKET);
            break;

        case SOCK_ESTABLISHED:
            if (getSn_IR(TCP_SOCKET) & Sn_IR_CON) {
                setSn_IR(TCP_SOCKET, Sn_IR_CON);  // clear flag
                RS485_2_Printf("Client connected!\r\n");
            }
						if (getSn_RX_RSR(TCP_SOCKET) > 0) {
							// Nh?n d? li?u
							int32_t recv_len = recv(TCP_SOCKET, tcp_buf, TCP_BUF_SIZE);
							if (recv_len > 0) {
									last_data_tick = micros();
									tcp_buf[recv_len] = '\0';
//									RS485_2_Printf("Received: %s\r\n", tcp_buf);
								

									
									char command[4];
									int lin_int,ang_int;
									if (sscanf((char *)tcp_buf, "%3s %d %d\r\n", command, &lin_int, &ang_int) == 3) {
                        if (strcmp(command, "VEL") == 0) {
													flag_task = 1;
													linear = (float) lin_int/1000.0;
													angular = (float) ang_int/1000.0;
												}
									}
								
									// Phan hoi
									char response[] = "STM32 ACK\r\n";
									send(TCP_SOCKET, (uint8_t*)response, strlen(response));
							}
						}
            break;

        case SOCK_CLOSE_WAIT:
            disconnect(TCP_SOCKET);
            break;
    }
}
/********************************************************************************/
void Ethernet_Error (int timeout_milisecond)
{
		if (micros() - last_data_tick > timeout_milisecond*1000)
		{
				flag_ethernet_error = 1;
			  RS485_2_Printf("Reconnecting to Client...\r\n");
			
		}
		else
		{
			  flag_ethernet_error = 0;
		}
}
/********************************************************************************/
