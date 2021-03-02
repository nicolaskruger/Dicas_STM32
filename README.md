# Dicas STM32

## :pushpin: Tabela de conteúdos

- [tipos de entrada](#tipos-de-entrada)
- [write pin](#write-pin)
- [read pin](#read-pin)
- [timer interrupt](#timer-interrupt)
- [Usando UART no linux](#Usando-UART-no-linux)
- [printf](#printf)
- [scanf](#scanf)
- [button interrupt](#button-interrupt)
- [config display](#config-display)
- [RTOS](#RTOS)
- [PWM](#PWM)
- [ADC](#ADC)

## tipos de entrada

- pullup : não precisa de resistores estado padrão 1, gnd para 0
- pulldown : não precisa de resistores estado padrão 0, vin para 1 vin ≃ 3,3V maior q isso queima a placa
- no -pull : precisa de resistores down w up dependem da comfiguração

~~~~code
 o vin
 |          
  ]push
 |------pin
[ ]
 |
 gnd
~~~~

## write pin

~~~~c
HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
~~~~

- GPIOx: porta do pino
- GPIO_Pin: numero do pino
- PinState: estado do pino
  - GPIO_PIN_SET: 1
  - GPIO_PIN_RESET: 0

## read pin

~~~~c
HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
~~~~

- GPIOx: porta do pino
- GPIO_Pin: numero do pino

## timer interrupt

para setar o timer:

`sys core ->NVIC-> tm<number> .... [checked]`

Pinout & Configuration

`Timer->TIM<number>->NVIC Settings->TIM<number> global interrupt [checked]`

veja a frequência do clock

Clock Configuration: clk

`Timer->TIM<number>->Paramter Settings`

- prescale : numero que ira dividir o clk

- counter period: quanto vai contar até disparar o clk

defina o prescale e counter period.

Em um clk de 2MHz caso queiramos um periode de chama de interrupção
de 1s
pode mos definis um percale de 2k, isso resutaris em um clk de 1kHz
com um periodo de 1ms para termos um periodo de 1 s devemos contar 1k vezes
assim teremos um counter period de 1000.

## Usando UART no linux

para configurar a usart basta configurar os pinos 
PA2 e PA3 para o modo usart

`connectivity -> USART2` 

- definir um baud rate 115200 Bits/s
- word length 8Bits

para transmitir:

~~~~c
HAL_UART_Transmit(&huart2,"<palavra>" ,tamnanho, tempo de espera);

HAL_UART_Receive(&huart2,c,1,1000);
                         | |   |->tempo
                         | |-> tamnanho
                         |-> variavel
~~~~

para acessar o terminal:

intalar minicom:

~~~~bash
sudo apt-get install minicom
~~~~

lançar o terminal:

~~~~bash
minicom -D /dev/ttyACM0 
~~~~

## printf 

para poder utilizar o print f presecisa emplementar:

~~~~c
int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart2, c, 1, 10);
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
}
~~~~

## scanf

para implementar o scanf

~~~~c
int __io_getchar(void)
{
  uint8_t ch = 0;
  // Clear the Overrun flag just before receiving the first character
  __HAL_UART_CLEAR_OREFLAG(&huart2);

  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  //HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int _read(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    *ptr++ = __io_getchar();
  }

    return len;
}
~~~~

## button interrupt

pino no mode GPIO_EXIT

`System core -> NVIC -> EXIT... [checked]`

implementar a função:

~~~~c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  ...
}
~~~~

## config display

`System Core -> SYS -> debug (serial wire) , timebase Source (TIM 1 || SysTick)`
~~~~code
Connectivity -> SPI1 -> Mode(transmit Only Master), Hardware Nss(disabel)
	      	|
	      	->Parameter Settings -> Prescale (16)
Pinout view -> PA0(output) => LCD_RST
	    -> PA3(output) => LCD_CE
	    -> PA4(output) => LCD_DC
# colar drivers na parta do projeto

./Core/Inc/file.h
./Core/Src/file.c

#main.c
#include "file.h"

LCD_Init();  
~~~~

## RTOS

~~~~code
Middleware -> FREERTOS -> interface(CMSIS_V1)
			-> mutex #cria mutex
			-> task and queues #cria queues
			-> conficureparams -> mutex(enable)
~~~~

## PWM

~~~~code
Timer -> TIM2 -> Clock Source(Internal Clock), channel1(PWM Generation CH1)
~~~~

## ADC

Analog -> ADC1 IN1

~~~~c
//exemplo leitor ad
int32_t Controller::adRead(ADC_HandleTypeDef* hadc){
  HAL_ADC_Start(hadc);
  while(HAL_ADC_PollForConversion(hadc, 5)!= HAL_OK){}
  return HAL_ADC_GetValue(hadc);
}

~~~~
