
#include <zephyr/zephyr.h>
#include <device.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <drivers/i2c.h>
#include <math.h>

#define MAX30102_ADDRESS  0x57

#define MAX30102_INT_STATUS_1 0x00
#define MAX30102_INT_STATUS_2 0x01
#define MAX30102_INT_ENABLE_1 0x02
#define MAX30102_INT_ENABLE_2 0x03
#define MAX30102_FIFO_WRITE_PTR 0x04
#define MAX30102_OVERFLOW_COUNT 0x05
#define MAX30102_FIFO_READ_PTR 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_LED1_PULSE_AMP 0x0C
#define MAX30102_LED2_PULSE_AMP 0x0D

#define MAX30102_PARTIDR 0xFF
#define MAX30102_RESET 0x40

#define MAX_BRIGHTNESS 255
int32_t n_ir_buffer_length;         //data length
uint32_t aun_ir_buffer[500];        //IR LED sensor data
uint32_t aun_red_buffer[500];       //Red LED sensor data
int32_t n_sp02;                     //SPO2 value
int8_t ch_spo2_valid;               //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;               //heart rate value
int8_t  ch_hr_valid;                //indicator to show if the heart rate calculation is valid
#define FS 100
#define BUFFER_SIZE  (FS* 5)
#define MA4_SIZE  4                 // DO NOT CHANGE
#define HAMMING_SIZE  5             // DO NOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))
const uint16_t auw_hamm[31]={ 41,    276,    512,    276,     41 }; //Hamm=  long16(512* hamming(5)');
//uch_spo2_table is computed as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
                            99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
                            100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
                            97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
                            90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
                            80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
                            66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
                            49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
                            28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
                            3, 2, 1 } ;
static  int32_t an_dx[ BUFFER_SIZE - MA4_SIZE]; // delta
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t  *pch_hr_valid);

uint16_t cyclePosition;
uint16_t sixRAM;
uint16_t nineRAM;
uint16_t lowerRAM = 0;
uint16_t upperRAM = 0;
double VRta = 0;
double AMB = 0;
double sensorTemp = 0;
float S = 0;
double VRto = 0;
double Sto = 0;
double TAdut = 0;
double ambientTempK = 0;
double objectTempCalc = 0;
double objectTemp = 0;

#define SLEEP_TIME_MS	2000


static int init_I2C(const struct device *i2c_dev)
{
    uint32_t dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;
    int ret;

    ret = i2c_configure(i2c_dev, dev_config);
    if(ret) printk("I2C: Error (%d)\n", ret);
    else printk("I2C: Ok (%d)\n", ret);
    return ret;
}

static int write_bytes(const struct device *i2c_dev, uint8_t *buf, uint32_t num_bytes, uint16_t addrDevice)
{
  struct i2c_msg msg;

  msg.buf = buf;
  msg.len = num_bytes;
  msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

  return i2c_transfer(i2c_dev, &msg, 1, addrDevice);
}

static int read_bytes(const struct device *i2c_dev, uint16_t addr, uint8_t *data, uint32_t num_bytes)
{
  uint8_t wr_addr[2];
  struct i2c_msg msgs[2];

  wr_addr[0] = (addr >> 8) & 0xFF;
  wr_addr[1] = addr & 0xFF;

  /* Setup I2C messages */

  /* Send the address to read from */
  msgs[0].buf = wr_addr;
  msgs[0].len = 2U;
  msgs[0].flags = I2C_MSG_WRITE;

  /* Read from device. STOP after this. */
  msgs[1].buf = data;
  msgs[1].len = num_bytes;
  msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

  return i2c_transfer(i2c_dev, &msgs[0], 2, 0x57);
}

// ---------- MAX30102 -----------
static int check_MAX30102(const struct device *i2c_dev)
{
    uint8_t partid[1] = {0x0};
    int ret;

    ret = i2c_burst_read(i2c_dev, MAX30102_ADDRESS, MAX30102_PARTIDR, partid, 1);    // Read part ID 
    if(ret) printk("MAX30102: Error - not present!\n");
    else
      {
      if (partid[0] == 0x15) printk("MAX30102: present! Part ID: %dd\n", partid[0]);
      else
        { 
        printk("MAX30102: Error! Part ID: %dd\n", partid[0]);
        ret = -99;
        }
      }
    return ret;
}

static int reset_MAX30102(const struct device *i2c_dev)     // This function resets the MAX30102
{
    uint8_t sendbuf[1] = {MAX30102_RESET};
    int ret;

    ret = i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, sendbuf[0]);
    if(ret) printk("MAX30102: Reset Error!\n");
    else printk("MAX30102: Reset ok!\n");
    return ret;
}

static int clearInt_MAX30102(const struct device *i2c_dev)     // Reads/clears the interrupt status register
{
    uint8_t IntStatus1[1] = {0x0};
    int ret;

    ret = i2c_burst_read(i2c_dev, MAX30102_ADDRESS, MAX30102_INT_STATUS_1, IntStatus1, 1);    
    if(ret) printk("MAX30102: Interrupt Status 1: Error!\n");
    else printk("MAX30102: Interrupt Status 1: Ok (%d)\n", IntStatus1[0]);
    return ret;
}

static int init_MAX30102(const struct device *i2c_dev)     // This function resets the MAX30102
{
    uint8_t buf[1] = {0x00};
    int ret, ret1 = 0;

    buf[0] = 0xC0;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_INT_ENABLE_1, buf[0]); // INTR setting
    if(ret)
      {
      ret1 = -99;
      printk("MAX30102: INTR setting: Error!\n");
      }
    else printk("MAX30102: INTR setting: ok!\n");

    buf[0] = 0x00;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_INT_ENABLE_2, buf[0]); // INTR setting
    if(ret) 
      {
      ret1 = -99;
      printk("MAX30102: INTR setting: Error!\n");
      }
    else printk("MAX30102: INTR setting: ok!\n");

    buf[0] = 0x00;
     ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_FIFO_WRITE_PTR, buf[0]); // FIFO_WR_PTR[4:0]
    if(ret)
      {
      ret1 = -99;      
      printk("MAX30102: FIFO_WR_PTR: Error!\n");
      }
    else printk("MAX30102: FIFO_WR_PTR: ok!\n");

    buf[0] = 0x00;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_OVERFLOW_COUNT, buf[0]);  // OVF_COUNTER[4:0]
    if(ret)
      {
      ret1 = -99;    
      printk("MAX30102: FIFO_WR_PTR: Error!\n");
      }
    else printk("MAX30102: FIFO_WR_PTR: ok!\n");

    buf[0] = 0x00;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_FIFO_READ_PTR, buf[0]); // FIFO_RD_PTR[4:0]
    if(ret)
      {
      ret1 = -99;
      printk("MAX30102: FIFO_RD_PTR: Error!\n");
      }
    else printk("MAX30102: FIFO_RD_PTR: ok!\n");

    buf[0] = 0x0F;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, buf[0]); // sample avg = 1, fifo rollover=false, fifo almost full = 17
    
    if(ret)
      {
      ret1 = -99;
      printk("MAX30102: FIFO CONFIG: Error!\n");
      }
    else printk("MAX30102: FIFO CONFIG: ok!\n");

    buf[0] = 0x03;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_MODE_CONFIG, buf[0]);// 0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    if(ret)
      {
      ret1 = -99;    
      printk("MAX30102: MODE CONFIG: Error!\n");
      }
    else printk("MAX30102: MODE CONFIG: ok!\n");

    buf[0] = 0x27;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, buf[0]);// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    if(ret)
      {
      ret1 = -99;    
      printk("MAX30102: SPO2 ADC: Error!\n");
      }
    else printk("MAX30102: SPO2 ADC: ok!\n");

    buf[0] = 0x24;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_LED1_PULSE_AMP, buf[0]);  // Choose value for ~ 7mA for LED1
    if(ret)
      {
      ret1 = -99;
      printk("MAX30102: LED1: Error!\n");
      }
    else printk("MAX30102: LED1: ok!\n");

    buf[0] = 0x24;
    ret =  i2c_reg_write_byte(i2c_dev, MAX30102_ADDRESS, MAX30102_LED2_PULSE_AMP, buf[0]);  // Choose value for ~ 7mA for LED2
    if(ret)
      {
      ret1 = -99;   
      printk("MAX30102: LED2: Error!\n");
      }
    else printk("MAX30102: LED2: ok!\n");

    return ret1;
}

bool maxim_max30102_read_reg(const struct device *i2c_dev, uint8_t uch_addr, uint8_t *puch_data)
{
  int ret;
  uint8_t buf[1] = {0x00};

  ret = i2c_burst_read(i2c_dev, MAX30102_ADDRESS, uch_addr, buf, 1);  
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

bool maxim_max30102_read_fifo(const struct device *i2c_dev, uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
  int ret;
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  char ach_i2c_data[6];
  
  //read and clear status register
  maxim_max30102_read_reg(i2c_dev, MAX30102_INT_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(i2c_dev, MAX30102_INT_STATUS_2, &uch_temp);

  ret = i2c_burst_read(i2c_dev, MAX30102_ADDRESS, MAX30102_REG_FIFO_DATA, ach_i2c_data, 6);
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


void main(void)
{

  int ret;
  uint8_t data[16];

  // -------------------------------------
  uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int i;
  int32_t n_brightness;
  float f_temp;
//   const struct device *IntMAX30102; 

//   IntMAX30102 = device_get_binding("I2C_1");
//   if (IntMAX30102 == NULL)
//   {
//     printk("IntMAX30102 Error: didn't find %s device\n");
//     return;
//   }

//   ret = gpio_pin_configure(IntMAX30102, SW3_GPIO_PIN, SW3_GPIO_FLAGS);
//   if (ret != 0)
//   {
//     //printk("Error %d: failed to configure %s pin %d\n", ret, SW3_GPIO_LABEL, SW3_GPIO_PIN);
//     return;
//   }

  k_msleep(1000);

  const struct device *i2c_dev;
  i2c_dev = device_get_binding("I2C_1");

    init_I2C(i2c_dev);

  //i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

  if (i2c_dev == NULL)
  {
    printk("Error: didn't find I2C_1 device\n");
    return;
  }
  
  // init_I2C(i2c_dev);            // I2C initialize


  // *************** MAX30102 *******************
  // Initialize MAX30102 Settings
  check_MAX30102(i2c_dev);      // Read part ID
  reset_MAX30102(i2c_dev);      // This function resets the MAX30102
  clearInt_MAX30102(i2c_dev);   // Reads/clears the interrupt status register
  init_MAX30102(i2c_dev);       // Initialize the MAX30102

  n_brightness=0;
  un_min=0x3FFFF;
  un_max=0;

  n_ir_buffer_length=500;

  // MAX30102 - read the first 500 samples, and determine the signal range
  for(i=0; i<n_ir_buffer_length; i++)
  {
    //while(gpio_pin_get(device_get_binding(PORT3), SW3_GPIO_PIN) == 0);  // wait until the interrupt pin asserts
    maxim_max30102_read_fifo(i2c_dev, (aun_red_buffer+i), (aun_ir_buffer+i));    // read from MAX30102 FIFO

    if(un_min>aun_red_buffer[i]) un_min=aun_red_buffer[i];    //update signal min
    if(un_max<aun_red_buffer[i]) un_max=aun_red_buffer[i];    //update signal max
    
    printk("red=%i, ir=%i\n", aun_red_buffer[i], aun_ir_buffer[i]);
  }
  un_prev_data=aun_red_buffer[i];

  //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while(1)
  {
      i=0;
      un_min=0x3FFFF;
      un_max=0;
      
      //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
      for(i=100;i<500;i++)
      {
          aun_red_buffer[i-100]=aun_red_buffer[i];
          aun_ir_buffer[i-100]=aun_ir_buffer[i];
          
          //update the signal min and max
          if(un_min>aun_red_buffer[i])
          un_min=aun_red_buffer[i];
          if(un_max<aun_red_buffer[i])
          un_max=aun_red_buffer[i];
      }
      
      //take 100 sets of samples before calculating the heart rate.
      for(i=400;i<500;i++)
      {
          un_prev_data=aun_red_buffer[i-1];
          maxim_max30102_read_fifo(i2c_dev, (aun_red_buffer+i), (aun_ir_buffer+i));
      
          if(aun_red_buffer[i]>un_prev_data)
          {
              f_temp=aun_red_buffer[i]-un_prev_data;
              f_temp/=(un_max-un_min);
              f_temp*=MAX_BRIGHTNESS;
              n_brightness-=(int)f_temp;
              if(n_brightness<0)
                  n_brightness=0;
          }
          else
          {
              f_temp=un_prev_data-aun_red_buffer[i];
              f_temp/=(un_max-un_min);
              f_temp*=MAX_BRIGHTNESS;
              n_brightness+=(int)f_temp;
              if(n_brightness>MAX_BRIGHTNESS)
                  n_brightness=MAX_BRIGHTNESS;
          }
          //send samples and calculation result to terminal program through UART
        printk("red=%i, ir=%i, HR=%i, HRvalid=%i, SpO2=%i, SPO2Valid=%i \n", aun_red_buffer[i], aun_ir_buffer[i], n_heart_rate, ch_hr_valid, n_sp02, ch_spo2_valid);
        k_msleep(10);
      }
      maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  
  }
}

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */



// ********** Algorithm - maxim_heart_rate_and_oxygen_saturation **********************
void maxim_sort_ascend(int32_t *pn_x,int32_t n_size) 
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/ 
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{
    
    int32_t i, j, n_old_npks, n_dist;
    
    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ){
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ){
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x, int32_t n_size, int32_t n_min_height)
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width;
    *pn_npks = 0;
    
    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*pn_npks) < 15 ){                            // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;        
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
    maxim_peaks_above_min_height( pn_locs, pn_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, pn_npks, pn_x, n_min_distance );
    *pn_npks = min( *pn_npks, n_max_num );
}

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t  *pch_hr_valid)
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
    uint32_t un_ir_mean ,un_only_once ;
    int32_t k ,n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count ,n_middle_idx;
    int32_t n_th1, n_npks,n_c_min;      
    int32_t an_ir_valley_locs[15] ;
    int32_t an_exact_ir_valley_locs[15] ;
    int32_t an_dx_peak_locs[15] ;
    int32_t n_peak_interval_sum;
    
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc; 
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
    int32_t an_ratio[5],n_ratio_average; 
    int32_t n_nume,  n_denom ;
    // remove DC of ir signal    
    un_ir_mean =0; 
    for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
    un_ir_mean =un_ir_mean/n_ir_buffer_length ;
    for (k=0 ; k<n_ir_buffer_length ; k++ )  an_x[k] =  pun_ir_buffer[k] - un_ir_mean ; 
    
    // 4 pt Moving Average
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        n_denom= ( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3]);
        an_x[k]=  n_denom/(int32_t)4; 
    }

    // get difference of smoothed IR signal
    
    for( k=0; k<BUFFER_SIZE-MA4_SIZE-1;  k++)
        an_dx[k]= (an_x[k+1]- an_x[k]);

    // 2-pt Moving Average to an_dx
    for(k=0; k< BUFFER_SIZE-MA4_SIZE-2; k++){
        an_dx[k] =  ( an_dx[k]+an_dx[k+1])/2 ;
    }
    
    // hamming window
    // flip wave form so that we can detect valley with peak detector
    for ( i=0 ; i<BUFFER_SIZE-HAMMING_SIZE-MA4_SIZE-2 ;i++){
        s= 0;
        for( k=i; k<i+ HAMMING_SIZE ;k++){
            s -= an_dx[k] *auw_hamm[k-i] ; 
                     }
        an_dx[i]= s/ (int32_t)1146; // divide by sum of auw_hamm 
    }

 
    n_th1=0; // threshold calculation
    for ( k=0 ; k<BUFFER_SIZE-HAMMING_SIZE ;k++){
        n_th1 += ((an_dx[k]>0)? an_dx[k] : ((int32_t)0-an_dx[k])) ;
    }
    n_th1= n_th1/ ( BUFFER_SIZE-HAMMING_SIZE);
    // peak location is acutally index for sharpest location of raw signal since we flipped the signal         
    maxim_find_peaks( an_dx_peak_locs, &n_npks, an_dx, BUFFER_SIZE-HAMMING_SIZE, n_th1, 8, 5 );//peak_height, peak_distance, max_num_peaks 

    n_peak_interval_sum =0;
    if (n_npks>=2){
        for (k=1; k<n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k]-an_dx_peak_locs[k -1]);
        n_peak_interval_sum=n_peak_interval_sum/(n_npks-1);
        *pn_heart_rate=(int32_t)(6000/n_peak_interval_sum);// beats per minutes
        *pch_hr_valid  = 1;
    }
    else  {
        *pn_heart_rate = -999;
        *pch_hr_valid  = 0;
    }
            
    for ( k=0 ; k<n_npks ;k++)
        an_ir_valley_locs[k]=an_dx_peak_locs[k]+HAMMING_SIZE/2; 


    // raw value : RED(=y) and IR(=X)
    // we need to assess DC and AC value of ir and red PPG. 
    for (k=0 ; k<n_ir_buffer_length ; k++ )  {
        an_x[k] =  pun_ir_buffer[k] ; 
        an_y[k] =  pun_red_buffer[k] ; 
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count =0; 
    for(k=0 ; k<n_npks ;k++){
        un_only_once =1;
        m=an_ir_valley_locs[k];
        n_c_min= 16777216;//2^24;
        if (m+5 <  BUFFER_SIZE-HAMMING_SIZE  && m-5 >0){
            for(i= m-5;i<m+5; i++)
                if (an_x[i]<n_c_min){
                    if (un_only_once >0){
                       un_only_once =0;
                   } 
                   n_c_min= an_x[i] ;
                   an_exact_ir_valley_locs[k]=i;
                }
            if (un_only_once ==0)
                n_exact_ir_valley_locs_count ++ ;
        }
    }
    if (n_exact_ir_valley_locs_count <2 ){
       *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
       *pch_spo2_valid  = 0; 
       return;
    }
    // 4 pt MA
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int32_t)4;
        an_y[k]=( an_y[k]+an_y[k+1]+ an_y[k+2]+ an_y[k+3])/(int32_t)4;
    }

    //using an_exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration ratio
    //finding AC/DC maximum of raw ir * red between two valley locations
    n_ratio_average =0; 
    n_i_ratio_count =0; 
    
    for(k=0; k< 5; k++) an_ratio[k]=0;
    for (k=0; k< n_exact_ir_valley_locs_count; k++){
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE ){             
            *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0; 
            return;
        }
    }
    // find max between two valley locations 
    // and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 

    for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
        n_y_dc_max= -16777216 ; 
        n_x_dc_max= - 16777216; 
        if (an_exact_ir_valley_locs[k+1]-an_exact_ir_valley_locs[k] >10){
            for (i=an_exact_ir_valley_locs[k]; i< an_exact_ir_valley_locs[k+1]; i++){
                if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i];n_x_dc_max_idx =i; }
                if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i];n_y_dc_max_idx=i;}
            }
            n_y_ac= (an_y[an_exact_ir_valley_locs[k+1]] - an_y[an_exact_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_exact_ir_valley_locs[k]); //red
            n_y_ac=  an_y[an_exact_ir_valley_locs[k]] + n_y_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k])  ; 
        
        
            n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw 
            n_x_ac= (an_x[an_exact_ir_valley_locs[k+1]] - an_x[an_exact_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_exact_ir_valley_locs[k]); // ir
            n_x_ac=  an_x[an_exact_ir_valley_locs[k]] + n_x_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k]); 
            n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw 
            n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
            n_denom= ( n_x_ac *n_y_dc_max)>>7;
            if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
            {   
                an_ratio[n_i_ratio_count]= (n_nume*100)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx= n_i_ratio_count/2;

    if (n_middle_idx >1)
        n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx ];

    if( n_ratio_average>2 && n_ratio_average <184){
        n_spo2_calc= uch_spo2_table[n_ratio_average] ;
        *pn_spo2 = n_spo2_calc ;
        *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
    }
    else{
        *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid  = 0; 
    }
}