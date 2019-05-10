// See LICENSE for license details.
#include <stdint.h>

#include <platform.h>

#include "common.h"

#define DEBUG
#include "kprintf.h"

#define MAX_CORES 8

#define PAYLOAD_SIZE	(26 << 11)

#ifndef TL_CLK
#error Must define TL_CLK
#endif

#define F_CLK TL_CLK

#include "gpt.h"

// GPT represents GUIDs with the first three blocks as little-endian
// c12a7328-f81f-11d2-ba4b-00a0c93ec93b
const gpt_guid gpt_guid_efi = {{
  0x28, 0x73, 0x2a, 0xc1, 0x1f, 0xf8, 0xd2, 0x11, 0xba, 0x4b, 0x00, 0xa0, 0xc9, 0x3e, 0xc9, 0x3b
}};
// 5b193300-fc78-40cd-8002-e86c45580b47
const gpt_guid gpt_guid_sifive_fsbl = {{
  0x00, 0x33, 0x19, 0x5b, 0x78, 0xfc, 0xcd, 0x40, 0x80, 0x02, 0xe8, 0x6c, 0x45, 0x58, 0x0b, 0x47
}};
// 2e54b353-1271-4842-806f-e436d6af6985
const gpt_guid gpt_guid_sifive_bare_metal = {{
  0x53, 0xb3, 0x54, 0x2e, 0x71, 0x12, 0x42, 0x48, 0x80, 0x6f, 0xe4, 0x36, 0xd6, 0xaf, 0x69, 0x85
}};

static volatile uint32_t * const spi = (void *)(SPI_CTRL_ADDR);

static inline uint8_t spi_xfer(uint8_t d) {
	int32_t r;
	REG32(spi, SPI_REG_TXFIFO) = d;
	do {
		r = REG32(spi, SPI_REG_RXFIFO);
	} while (r < 0);
	return r;
}

static inline uint8_t sd_dummy(void) {
	return spi_xfer(0xFF);
}

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
	unsigned long n;
	uint8_t r;

	REG32(spi, SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
	sd_dummy();
	spi_xfer(cmd);
	spi_xfer(arg >> 24);
	spi_xfer(arg >> 16);
	spi_xfer(arg >> 8);
	spi_xfer(arg);
	spi_xfer(crc);

	n = 1000;
	do {
	   r = sd_dummy();
	   if (!(r & 0x80)) {
//	      dprintf("sd:cmd: %hx\r\n", r);
	      goto done;
	   }
	} while (--n > 0);
	kputs("sd_cmd: timeout");
done:
	return r;
}

static inline void sd_cmd_end(void) {
	sd_dummy();
	REG32(spi, SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
}

static void sd_poweron(void) {
	long i;
	REG32(spi, SPI_REG_SCKDIV) = (F_CLK / 300000UL);
	REG32(spi, SPI_REG_CSMODE) = SPI_CSMODE_OFF;
	for (i = 10; i > 0; i--) { sd_dummy(); }
	REG32(spi, SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
}

static int sd_cmd0(void) {
	int rc;
	dputs("CMD0");
	rc = (sd_cmd(0x40, 0, 0x95) != 0x01);
	sd_cmd_end();
	return rc;
}

static int sd_cmd8(void) {
	int rc;
	dputs("CMD8");
	rc = (sd_cmd(0x48, 0x000001AA, 0x87) != 0x01);
	sd_dummy(); /* command version; reserved */
	sd_dummy(); /* reserved */
	rc |= ((sd_dummy() & 0xF) != 0x1); /* voltage */
	rc |= (sd_dummy() != 0xAA); /* check pattern */
	sd_cmd_end();
	return rc;
}

static void sd_cmd55(void) {
	sd_cmd(0x77, 0, 0x65);
	sd_cmd_end();
}

static int sd_acmd41(void) {
	uint8_t r;
	dputs("ACMD41");
	do {
	   sd_cmd55();
	   r = sd_cmd(0x69, 0x40000000, 0x77); /* HCS = 1 */
	} while (r == 0x01);
	return (r != 0x00);
}

static int sd_cmd58(void) {
	int rc;
	dputs("CMD58");
	rc = (sd_cmd(0x7A, 0, 0xFD) != 0x00);
	rc |= ((sd_dummy() & 0x80) != 0x80); /* Power up status */
	sd_dummy();
	sd_dummy();
	sd_dummy();
	sd_cmd_end();
	return rc;
}

static int sd_cmd16(void) {
	int rc;
	dputs("CMD16");
	rc = (sd_cmd(0x50, 0x200, 0x15) != 0x00);
	sd_cmd_end();
	return rc;
}

#define SPIN_SHIFT	6
#define SPIN_UPDATE(i)	(!((i) & ((1 << SPIN_SHIFT)-1)))
#define SPIN_INDEX(i)	(((i) >> SPIN_SHIFT) & 0x3)

static const char spinner[] = { '-', '/', '|', '\\' };

#define GPT_BLOCK_SIZE 512

#define SD_COPY_ERROR_CMD18 1
#define SD_COPY_ERROR_CMD18_CRC 2

#define SD_CMD_STOP_TRANSMISSION 12
#define SD_CMD_READ_BLOCK_MULTIPLE 18

// Data token for commands 17, 18, 24
#define SD_DATA_TOKEN 0xfe
// Command frame starts by asserting low and then high for first two clock edges
#define SD_CMD(cmd) (0x40 | (cmd))

static uint8_t crc7(uint8_t prev, uint8_t in) {
  // CRC polynomial 0x89
  uint8_t remainder = prev & in;
  remainder ^= (remainder >> 4) ^ (remainder >> 7);
  remainder ^= remainder << 4;
  return remainder & 0x7f;
}

static uint16_t crc16(uint16_t crc, uint8_t data) {
  // CRC polynomial 0x11021
  crc = (uint8_t)(crc >> 8) | (crc << 8);
  crc ^= data;
  crc ^= (uint8_t)(crc >> 4) & 0xf;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

int sd_copy(void* dst, uint32_t src_lba, size_t size) {
  volatile uint8_t *p = dst;
  long i = size;
  int rc = 0;

  uint8_t crc = 0;
  crc = crc7(crc, SD_CMD(SD_CMD_READ_BLOCK_MULTIPLE));
  crc = crc7(crc, src_lba >> 24);
  crc = crc7(crc, (src_lba >> 16) & 0xff);
  crc = crc7(crc, (src_lba >> 8) & 0xff);
  crc = crc7(crc, src_lba & 0xff);
  crc = (crc << 1) | 1;
  if (sd_cmd(SD_CMD(SD_CMD_READ_BLOCK_MULTIPLE), src_lba, crc) != 0x00) {
    sd_cmd_end();
    return SD_COPY_ERROR_CMD18;
  } do {
    uint16_t crc, crc_exp;
    long n;

    crc = 0;
    n = 512;
    while (sd_dummy() != SD_DATA_TOKEN);
    do {
      uint8_t x = sd_dummy();
      *p++ = x;
      crc = crc16(crc, x);
    } while (--n > 0);

    crc_exp = ((uint16_t)sd_dummy() << 8);
    crc_exp |= sd_dummy();

    if (crc != crc_exp) {
      rc = SD_COPY_ERROR_CMD18_CRC;
      break;
    }
    if ((i % 2000) == 0) { 
      kputs(".");
    }
  } while (--i > 0);

  sd_cmd(SD_CMD(SD_CMD_STOP_TRANSMISSION), 0, 0x01);
  sd_cmd_end();
  return rc;
}

int sd_copy_partition(uint32_t src_lba, size_t size) {
  volatile uint8_t *p = (void *)(PAYLOAD_DEST+0x10000000); //fix at 0x90000000
  long i = size;
  int rc = 0;

  uint8_t crc = 0;
  crc = crc7(crc, SD_CMD(SD_CMD_READ_BLOCK_MULTIPLE));
  crc = crc7(crc, src_lba >> 24);
  crc = crc7(crc, (src_lba >> 16) & 0xff);
  crc = crc7(crc, (src_lba >> 8) & 0xff);
  crc = crc7(crc, src_lba & 0xff);
  crc = (crc << 1) | 1;
  
  dputs("CMD18");
  kprintf("LOADING  ");

  if (sd_cmd(SD_CMD(SD_CMD_READ_BLOCK_MULTIPLE), src_lba, crc) != 0x00) {
    sd_cmd_end();
    return SD_COPY_ERROR_CMD18;
  } do {
    uint16_t crc, crc_exp;
    long n;

    crc = 0;
    n = 512;
    while (sd_dummy() != SD_DATA_TOKEN);
    do {
      uint8_t x = sd_dummy();
      *p++ = x;
      crc = crc16(crc, x);
    } while (--n > 0);

    crc_exp = ((uint16_t)sd_dummy() << 8);
    crc_exp |= sd_dummy();

    if (crc != crc_exp) {
      kputs("\b- CRC mismatch ");
      rc = SD_COPY_ERROR_CMD18_CRC;
      break;
    }
    if (SPIN_UPDATE(i)) {
      kputc('\b');
      kputc(spinner[SPIN_INDEX(i)]);
    }
  } while (--i > 0);
  sd_cmd_end();

  sd_cmd(SD_CMD(SD_CMD_STOP_TRANSMISSION), 0, 0x01);
  sd_cmd_end();
  kputs("\b ");
  return rc;
}

static gpt_partition_range find_sd_gpt_partition (
  uint64_t partition_entries_lba,
  uint32_t num_partition_entries,
  uint32_t partition_entry_size,
  const gpt_guid* partition_type_guid,
  void* block_buf  // Used to temporarily load blocks of SD card
) {
  // Exclusive end
  uint64_t partition_entries_lba_end = (
    partition_entries_lba +
    (num_partition_entries * partition_entry_size + GPT_BLOCK_SIZE - 1) / GPT_BLOCK_SIZE
  );
  uint32_t num_entries;
  num_entries = GPT_BLOCK_SIZE / partition_entry_size;
  kprintf("Number of partition: 0x%x\r\n",num_entries);
  for (uint64_t i = partition_entries_lba; i < partition_entries_lba_end; i++) {
    sd_copy(block_buf, i, 1);
    gpt_partition_range range = gpt_find_partition_by_guid (
      block_buf, partition_type_guid, num_entries
    );
    if (gpt_is_valid_partition_range(range)) {
      kprintf("Partition range: 0x%x to 0x%x\r\n",range.first_lba,range.last_lba);
      return range;
    }
  }
  return gpt_invalid_partition_range();
}

int main(void) {
  REG32(uart, UART_REG_TXCTRL) = UART_TXEN;

  kputs("INIT");
  sd_poweron();
  if (sd_cmd0() || sd_cmd8() || sd_acmd41() || sd_cmd58() || sd_cmd16()) {
         kputs("ERROR");
         return 1;
      }
  
  uint8_t gpt_buf[GPT_BLOCK_SIZE];
  int error;

  dputs("CMD18");
  REG32(spi, SPI_REG_SCKDIV) = (F_CLK / 16666666UL);
  
  kputs("sd_copy: gpt_header");
  error = sd_copy(gpt_buf, GPT_HEADER_LBA, 1);
  
  if (error) { kputs("Fail at sd_copy: gpt_header"); return 1; }

  kputs("finding partition");
  gpt_partition_range part_range;
  {
    // header will be overwritten by find_sd_gpt_partition(), so locally
    // scope it.
    gpt_header* header = (gpt_header*) gpt_buf;
    part_range = find_sd_gpt_partition(
      header->partition_entries_lba,
      header->num_partition_entries,
      header->partition_entry_size,
      //&gpt_guid_sifive_bare_metal,	// bbl guid
      &gpt_guid_sifive_fsbl,		// fsbl guid
      gpt_buf
    );
  }
  
  if (!gpt_is_valid_partition_range(part_range)) {
    kputs("fail at finding partition"); return 1;
  }

  kputs("sd_copy: data");
  error = sd_copy_partition(
    part_range.first_lba,
    part_range.last_lba + 1 - part_range.first_lba
    //PAYLOAD_SIZE
  );

  if (error) { kputs("fail at sd_copy: data"); return 1; }

  kputs("BOOT");

  __asm__ __volatile__ ("fence.i" : : : "memory");
  return 0;
}
