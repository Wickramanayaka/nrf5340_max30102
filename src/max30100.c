#include "max30102.h"

const struct device *dev_i2c;

int MAX30102_check(const struct device *dev_i2c)
{
    uint8_t partid[1] = {0x0};
    int ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, MAX30102_PARTID, partid, 1);    // Read part ID 
	
    if(ret) printk("MAX30102 not present!\n");
    else
      {
      if (partid[0] == 0x15) printk("MAX30102: present! Part ID: %dd\n", partid[0]);
      else
        { 
        printk("MAX30102: Error! Part ID: %dd\n", partid[0]);
        ret = 0;
        }
      }
    return ret;
}

int MAX30102_reset(const struct device *dev_i2c)     // This function resets the MAX30102
{
    uint8_t sendbuf[1] = {MAX30102_RESET};
    int ret = i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, sendbuf[0]);
	
    if(ret) printk("Reset Error!\n");
    else printk("Reset done!\n");
    return ret;
}

int MAX30102_clear(const struct device *dev_i2c)     // Reads/clears the interrupt status register
{
    uint8_t IntStatus1[1] = {0x0};
    int ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, MAX30102_INT_STATUS_1, IntStatus1, 1);
    
    if(ret) printk("MAX30102: Interrupt Status 1: Error!\n");
    else printk("MAX30102: Interrupt Status 1: Ok (%d)\n", IntStatus1[0]);
    return ret;
}

int MAX30102_config(const struct device *dev_i2c)     
{
    uint8_t buf[1] = {0x00};
    int ret, ret1 = 0;

    buf[0] = 0xC0;
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_INT_ENABLE_1, buf[0]);
    if(ret)
      {
      ret1 = 0;
      }

    buf[0] = 0x00;
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_INT_ENABLE_2, buf[0]); 
    if(ret) 
      {
      ret1 = 0;
      }

    buf[0] = 0x00;
     ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_FIFO_WRITE_PTR, buf[0]); 
    if(ret)
      {
      ret1 = 0;      
      }

    buf[0] = 0x00;
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_OVERFLOW_COUNT, buf[0]);  
    if(ret)
      {
      ret1 = 0;    
      }

    buf[0] = 0x00;
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_FIFO_READ_PTR, buf[0]); 
    if(ret)
      {
      ret1 = 0;
      }

    buf[0] = 0x0F; // sample avg = 1, fifo rollover=false, fifo almost full = 17
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, buf[0]); 
    
    if(ret)
      {
      ret1 = 0;
      }

    buf[0] = 0x03; // 0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, buf[0]);
    if(ret)
      {
      ret1 = 0;    
      }

    buf[0] = 0x27; // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, buf[0]);
    if(ret)
      {
      ret1 = 0;    
      }

    buf[0] = 0x24; // LED1 current ~ 7mA 
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_LED1_PULSE_AMP, buf[0]);  
    if(ret)
      {
      ret1 = 0;
      }

    buf[0] = 0x24; // LED2 current ~ 7mA 
    ret =  i2c_reg_write_byte(dev_i2c, MAX30102_ADDRESS, MAX30102_LED2_PULSE_AMP, buf[0]);  
    if(ret)
      {
      ret1 = 0;   
      }
    return ret1;
}

bool MAX30102_read_reg(const struct device *dev_i2c, uint8_t uch_addr, uint8_t *puch_data)
{
  int ret;
  uint8_t buf[1] = {0x00};

  ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, uch_addr, buf, 1);  
  if(ret)
  {  
    printk("MAX30102: Error!\n");
    return false;
  }
  else 
  {
    *puch_data = buf[0];
    return true;
  }
}

bool MAX30102_read_fifo(const struct device *dev_i2c, uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
  int ret;
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  char ach_i2c_data[6];
  
  //read and clear status register
  MAX30102_read_reg(dev_i2c, MAX30102_INT_STATUS_1, &uch_temp);
  MAX30102_read_reg(dev_i2c, MAX30102_INT_STATUS_2, &uch_temp);

  ret = i2c_burst_read(dev_i2c, MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA, ach_i2c_data, 6);
  if(ret) printk("MAX30102: maxim_max30102_read_fifo Error!\n");

  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;
  *pun_ir_led&=0x03FFFF;

  return true;
}