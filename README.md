# **Proyecto_Oximetro_Pulsos**
*Repositorio destinado para el proyecto de la materia Técnicas Digitales 3*

## Se encuentra disponible en la carpeta "Documentos" el preinforme del proyecto.
## Se agrega el código desarollado con STM32CubeIDE, con las siguientes configuraciones:

-UART: Se utilizará para la comunicación con el módulo bluetooth HM10.
	-PA2: TX
	-PA3: RX

-SPI: Se utilizará para la comunicación con el módulo R/W de tarjetas Micro SD
	-PB12: CSS
	-PB13: SCK
	-PB14: MISO
	-PB15: MOSI

-I2C1: Se utilizará para la comunicación con el módulo oxímetro MAX30100
	-PB6: SCL
	-PB7: SDA

-I2C2: Se utilizará para la comunicación con la pantalla OLED SSD1306
	-PB10: SCL
	-PB11: SDA

-PULSADOR: Se utilizará para comenzar con la medición.
	-PC14: PULL-UP