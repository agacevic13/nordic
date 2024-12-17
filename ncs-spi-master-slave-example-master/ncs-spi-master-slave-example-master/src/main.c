/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "mfrc522.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
#define MY_SPI_MASTER_CS_DT_SPEC SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master))

#define MY_SPI_SLAVE  DT_NODELABEL(my_spi_slave)
LOG_MODULE_REGISTER(my_module, LOG_LEVEL_INF);
// SPI master functionality
const struct device *spi_dev;
//static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);
	static uint8_t tx_buffer[2];
	static uint8_t rx_buffer[1];
	static uint8_t tx_r_buffer[1];
	static const uint8_t m_length = 1;
	static volatile bool spi_xfer_done;
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	const struct spi_buf tx_r_buff ={
		.buf = &tx_r_buffer,
		.len = 1
	};

	const struct spi_buf_set tx_r = {
		.buffers = &tx_r_buff,
		.count = 1
	};

static void spi_init(void)
{
	spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi_dev)) {
		printk("SPI master device not ready!\n");
	}
	struct gpio_dt_spec spim_cs_gpio = MY_SPI_MASTER_CS_DT_SPEC;
	if(!device_is_ready(spim_cs_gpio.port)){
		printk("SPI master chip select device not ready!\n");
	}
}

static struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 4000000,
	.slave = 0,
	.cs = {.gpio = MY_SPI_MASTER_CS_DT_SPEC, .delay = 0},
};

// static int spi_write_test_msg(void)
// {
// 	static uint8_t counter = 0;
// 	static uint8_t tx_buffer[2];
// 	static uint8_t rx_buffer[2];

// 	const struct spi_buf tx_buf = {
// 		.buf = tx_buffer,
// 		.len = sizeof(tx_buffer)
// 	};
// 	const struct spi_buf_set tx = {
// 		.buffers = &tx_buf,
// 		.count = 1
// 	};

// 	struct spi_buf rx_buf = {
// 		.buf = rx_buffer,
// 		.len = sizeof(rx_buffer),
// 	};
// 	const struct spi_buf_set rx = {
// 		.buffers = &rx_buf,
// 		.count = 1
// 	};

// 	// Update the TX buffer with a rolling counter
// 	tx_buffer[0] = counter++;
// 	printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

// 	// Reset signal
// 	k_poll_signal_reset(&spi_done_sig);
	
// 	// Start transaction
// 	int error = spi_transceive_signal(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
// 	if(error != 0){
// 		printk("SPI transceive error: %i\n", error);
// 		return error;
// 	}

// 	// Wait for the done signal to be raised and log the rx buffer
// 	int spi_signaled, spi_result;
// 	do{
// 		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
// 	} while(spi_signaled == 0);
// 	printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
// 	return 0;
// }
/*
// SPI slave functionality
const struct device *spi_slave_dev;
static struct k_poll_signal spi_slave_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_slave_done_sig);

static const struct spi_config spi_slave_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,
	.frequency = 4000000,
	.slave = 0,
};

static void spi_slave_init(void)
{
	spi_slave_dev = DEVICE_DT_GET(MY_SPI_SLAVE);
	if(!device_is_ready(spi_slave_dev)) {
		printk("SPI slave device not ready!\n");
	}
	else{
		printk("Spi slave device ISS ready\n");
	}
}

static uint8_t slave_tx_buffer[2];
static uint8_t slave_rx_buffer[2];
static int spi_slave_write_test_msg(void)
{
	static uint8_t counter = 0;


	const struct spi_buf s_tx_buf = {
		.buf = slave_tx_buffer,
		.len = sizeof(slave_tx_buffer)
	};
	const struct spi_buf_set s_tx = {
		.buffers = &s_tx_buf,
		.count = 1
	};

	struct spi_buf s_rx_buf = {
		.buf = slave_rx_buffer,
		.len = sizeof(slave_rx_buffer),
	};
	const struct spi_buf_set s_rx = {
		.buffers = &s_rx_buf,
		.count = 1
	};

	// Update the TX buffer with a rolling counter
	slave_tx_buffer[1] = counter++;
	printk("SPI SLAVE TX: 0x%.2x, 0x%.2x\n", slave_tx_buffer[0], slave_tx_buffer[1]);

	// Reset signal
	k_poll_signal_reset(&spi_slave_done_sig);
	
	// Start transaction
	int error = spi_transceive_signal(spi_slave_dev, &spi_slave_cfg, &s_tx, &s_rx, &spi_slave_done_sig);
	if(error != 0){
		printk("SPI slave transceive error: %i\n", error);
		return error;
	}
	return 0;
}

static int spi_slave_check_for_message(void)
{
	int signaled, result;
	k_poll_signal_check(&spi_slave_done_sig, &signaled, &result);
	if(signaled != 0){
		return 0;
	}
	else return -1;
}
*/
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void PCD_WriteRegister(PCD_Register reg, byte value) {
	// Update the TX buffer with a rolling counter
	tx_buffer[0] = reg & 0x7E; //msb 0 za pisanje
	tx_buffer[1] = value;
	printk("SPI REGISTAR: 0x%.2x, tx value: 0x%.2x\n", tx_buffer[0], tx_buffer[1]);
    // Podešavanje adrese registra

    memset(rx_buffer, 0, m_length);
    // Šaljemo podatke putem SPI

	int error = spi_write(spi_dev, &spi_cfg, &tx);
	if(error != 0){
		printk("SPI write error: %i\n", error);
		return error;
	}
}
void PCD_WriteRegister_long(PCD_Register reg, byte count, byte *values){
		
        tx_buffer[0] = reg & 0x7E;
      
        

	for (byte index = 0; index < count; index++) {
          
		  tx_buffer[1] = values[index];
          spi_write(spi_dev, &spi_cfg, &tx);     //ovoo popravi, ne moze ovako
          
	}
}

byte PCD_ReadRegister(PCD_Register reg){
	tx_r_buffer[0]= (0x80 | (reg & 0x7E));


    spi_write(spi_dev, &spi_cfg, &tx_r);
	//memset(rx_buffer, 0, m_length);
	//printk("register is set for read");
    spi_read(spi_dev, &spi_cfg, &rx);

	return rx_buffer[0];
	
    
}
//ovu funkciju napisi drugacije
void PCD_ReadRegister_long(PCD_Register reg, byte count, byte *values, byte rxAlign){
	if (count == 0) {
		return;
	}
        tx_r_buffer[0] = 0x80 | (reg& 0x7E);				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
		byte index = 0;	
        count--;
    
       spi_write(spi_dev, &spi_cfg, &tx_r);

        if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		byte mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
        memset(rx_buffer, 0, m_length);

         spi_read(spi_dev, &spi_cfg, &rx);

        byte value = rx_buffer[0];
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}

        while (index < count) {
        memset(rx_buffer, 0, m_length);
        spi_read(spi_dev, &spi_cfg, &rx);
        values[index] = rx_buffer[0];
        index++;
	}
        memset(rx_buffer, 0, m_length);
     
        uint8_t _reg = 0x00;
       spi_read(spi_dev, &spi_cfg, &rx);

        values[index] = rx_buffer[0];

}
void PCD_ClearRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
				byte mask			///< The bits to clear.
									  ) {
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()

void PCD_SetRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
				byte mask			///< The bits to set.
									) { 
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask

} // End PCD_SetRegisterBitMask()


StatusCode PCD_CalculateCRC(	byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                byte length,	///< In: The number of bytes to transfer.
                                byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
) {
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_long(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		byte n = PCD_ReadRegister(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadRegister(CRCResultRegL);
			result[1] = PCD_ReadRegister(CRCResultRegH);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()

void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
                value = PCD_ReadRegister(TxControlReg);
	}
} // End PCD_AntennaOn()
StatusCode PICC_RequestA(byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
                         byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
			) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()
void printf_value(uint8_t count, uint8_t *values){
    for (uint8_t index = 0; index < count; index++) {
        printk("%p - %x\n",&values[index],values[index]);
    }
}
StatusCode PCD_CommunicateWithPICC(	byte command,		///< The command to execute. One of the PCD_Command enums.
                                        byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
                                        byte *sendData,		///< Pointer to the data to transfer to the FIFO.
                                        byte sendLen,		///< Number of bytes to transfer to the FIFO.
                                        byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
                                        byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                        byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                        byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                        bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
 ) {
	// Prepare values for BitFramingReg
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	byte mcmd = PCD_ReadRegister(CommandReg);
        PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
        PCD_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
        PCD_WriteRegister_long(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
        PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
        PCD_WriteRegister(CommandReg, command);				// Execute the command	
        if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	for (i = 2000; i > 0; i--) {

		byte n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
//		printk("n:%x",n);
                if (n & waitIRq) {					// One of the interrupts that signal success has been set.
                        break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
                        return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
        printk("STATUS_TIMEOUT2\n");
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	byte _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		byte n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister_long(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

StatusCode PCD_TransceiveData(	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
                                        byte sendLen,		///< Number of bytes to transfer to the FIFO.
                                        byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
                                        byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                        byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
                                        byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                        bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
 ) {
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()
StatusCode PICC_REQA_or_WUPA(	byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
				byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
				byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
) {
	byte validBits;
	StatusCode status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()
 
bool PICC_IsNewCardPresent() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);
	StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
    printk("result of new card:%x\n",result);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()
void PCD_Reset() {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);
	uint8_t count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
	   k_msleep(50);
	} while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()
bool PCD_PerformSelfTest() {
	// This follows directly the steps outlined in 16.1.1
	// 1. Perform a soft reset.
	PCD_Reset();
	
	// 2. Clear the internal buffer by writing 25 bytes of 00h
	byte ZEROES[25] = {0x00};
	PCD_WriteRegister(FIFOLevelReg, 0x80);		// flush the FIFO buffer

	PCD_WriteRegister_long(FIFODataReg, 25, ZEROES);	// write 25 bytes of 00h to FIFO

	PCD_WriteRegister(CommandReg, PCD_Mem);		// transfer to internal buffer
	
	// 3. Enable self-test
	PCD_WriteRegister(AutoTestReg, 0x09);
	
	// 4. Write 00h to FIFO buffer
	PCD_WriteRegister(FIFODataReg, 0x00);
	
	// 5. Start self-test by issuing the CalcCRC command
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);
	
	// 6. Wait for self-test to complete
	byte n;
	for (uint8_t i = 0; i < 0xFF; i++) {
		// The datasheet does not specify exact completion condition except
		// that FIFO buffer should contain 64 bytes.
		// While selftest is initiated by CalcCRC command
		// it behaves differently from normal CRC computation,
		// so one can't reliably use DivIrqReg to check for completion.
		// It is reported that some devices does not trigger CRCIRq flag
		// during selftest.
		n = PCD_ReadRegister(FIFOLevelReg);
		if (n >= 64) {
                printk("nnn:%x",n);
                
			break;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
	
	// 7. Read out resulting 64 bytes from the FIFO buffer.
	byte result[64];
	PCD_ReadRegister_long(FIFODataReg, 64, result, 0);
	
	// Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
	PCD_WriteRegister(AutoTestReg, 0x00);
	
	// Determine firmware version (see section 9.3.4.8 in spec)
	byte version = PCD_ReadRegister(VersionReg);
	
	// Pick the appropriate reference values
	const byte *reference;
	printk("version:%x",version);
	
	// Verify that the results match up to our expectations
	for (uint8_t i = 0; i < 64; i++) {
printk("result[%d]:%x",i,result[i]);

//		if (result[i] != pgm_read_byte(&(reference[i]))) {
//			return false;
//		}
	}
	printk("Test passed; all is good");
	// Test passed; all is good.
	return true;
} // End PCD_PerformSelfTest()


int main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	spi_init();

	//PCD_Reset();
    
     PCD_PerformSelfTest();
//   uint8_t v = PCD_ReadRegister(VersionReg);
//       LOG_INF("version:%x",v);
// 	// Reset baud rates
// 	PCD_WriteRegister(TxModeReg, 0x00);
// 	PCD_WriteRegister(RxModeReg, 0x00);
// 	// Reset ModWidthReg
// 	PCD_WriteRegister(ModWidthReg, 0x26);
// 	// When communicating with a PICC we need a timeout if something goes wrong.
// 	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
// 	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
// 	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
// 	v=PCD_ReadRegister(TModeReg);
//     LOG_INF("TModeReg:%x",v);
//     PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
// 	v=PCD_ReadRegister(TPrescalerReg);
//     LOG_INF("TPrescalerReg:%x",v);
//     PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
// 	v=PCD_ReadRegister(TPrescalerReg);
//     LOG_INF("TPrescalerReg:%x",v); 
//     PCD_WriteRegister(TReloadRegH, 0x04);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
// 	PCD_WriteRegister(TReloadRegL, 0xE8);	
// 	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the  register setting
// 	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
// 	PCD_AntennaOn();
//     //NRF_LOG_FLUSH();
    

#if 0
	spi_slave_init();

	printk("SPI master/slave example started\n");
	
	spi_slave_write_test_msg();

	while (1) {
		spi_write_test_msg();
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);

		if(spi_slave_check_for_message() == 0){
			// Print the last received data
			printk("SPI SLAVE RX: 0x%.2x, 0x%.2x\n", slave_rx_buffer[0], slave_rx_buffer[1]);
			
			// Prepare the next SPI slave transaction
			spi_slave_write_test_msg();
		}
	}
#endif
	return 0;
}
