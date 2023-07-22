#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

static volatile bool send_client_side_to_host = false;

static volatile bool corruption_armed = true;
static volatile uint32_t targeted_arbid = 0xff;

static volatile bool notify_fault_injected = false;
static volatile bool notify_fault_detected = false;

static volatile bool notify_fault_corrected = false;
static volatile bool next_message_retransmission = false;


uint16_t bits_read = 0; 
uint8_t  last_bit = 0; 
uint8_t  same_bits_count = 0; 
uint32_t arbid;

uint16_t msg_byte = 0; 
uint8_t  message[8]; 
uint8_t  extended_arbid = 0;
uint8_t  msg_len = 0;
uint32_t can_time_between_bits_us;
uint32_t can_initial_delay_us;
static volatile uint8_t frame_done = 0;
static volatile uint32_t can_rx_crc;
static uint32_t can_baud = 500000;
static volatile uint8_t tx_bit_count = 0;
static volatile uint32_t frames_seen = 0;
static bool client_is_receiver = true;
static volatile uint32_t targeted_rs485_id = 0xFF;


void (*end_of_frame_callback)(void) = NULL;

//#define CAN_INITIAL_DELAY (43040ns)
//4912
//#define CAN_INITIAL_DELAY (2949ns)
#define CAN_INITIAL_DELAY (2800ns)

//9824ns
#define CAN_DELAY (8138ns)
//#define CAN_DELAY (86081ns)

#define INTERFACE_ID 0
#define KILL_SWITCH_ID 8

#define CAN_BUS_RX_HOST_SIDE 2
#define CAN_BUS_TX_HOST_SIDE 3
#define CAN_BUS_RX_CLIENT_SIDE 21
#define CAN_BUS_TX_CLIENT_SIDE 20
#define CAN_BUS_SILENT_MODE 4
#define LED_PIN 13

#define RS485_RX_HOST 7
#define RS485_TX_HOST 8
#define RS485_HOST Serial2
#define RS485_DRIVER_EN_HOST 10
#define RS485_RECEIVER_EN_HOST 11
#define RS485_TERM_EN_HOST 12

#define RS485_RX_CLIENT 15
#define RS485_TX_HOST_CLIENT 14
#define RS485_CLIENT Serial3
#define RS485_DRIVER_EN_CLIENT 16
#define RS485_RECEIVER_EN_CLIENT 17
#define RS485_TERM_EN_CLIENT 18

#define RS485_START_BYTE 0x55

#define BAUDRATE_RS485 115200

OneShotTimer t1(GPT1);
OneShotTimer t2(GPT2);


typedef enum{
  SYNC,
  ID,
  SIZE,
  DATA
} state_rs485_receive_t;


static void sample_callback(void);
static void can_start_of_frame_detecte_handle(void);
void setup() {
  pinMode(CAN_BUS_RX_HOST_SIDE, INPUT);
  pinMode(CAN_BUS_TX_HOST_SIDE, OUTPUT);
  pinMode(CAN_BUS_RX_CLIENT_SIDE, INPUT);
  pinMode(CAN_BUS_TX_CLIENT_SIDE, OUTPUT);
  pinMode(CAN_BUS_SILENT_MODE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(RS485_DRIVER_EN_HOST, OUTPUT);
  pinMode(RS485_RECEIVER_EN_HOST, OUTPUT);
  pinMode(RS485_TERM_EN_HOST, OUTPUT);

  pinMode(RS485_DRIVER_EN_CLIENT, OUTPUT);
  pinMode(RS485_RECEIVER_EN_CLIENT, OUTPUT);
  pinMode(RS485_TERM_EN_CLIENT, OUTPUT);


  RS485_CLIENT.begin(BAUDRATE_RS485);
  RS485_CLIENT.transmitterEnable(RS485_DRIVER_EN_CLIENT);
  RS485_HOST.begin(BAUDRATE_RS485);
  RS485_HOST.transmitterEnable(RS485_DRIVER_EN_HOST);

  digitalWrite(RS485_TERM_EN_HOST,1);
  digitalWrite(RS485_RECEIVER_EN_HOST,0);

  digitalWrite(RS485_TERM_EN_CLIENT,1);
  digitalWrite(RS485_RECEIVER_EN_CLIENT,0);

  pinMode(0,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );
  attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
  
  t1.begin(sample_callback);
  Serial.begin(9600);
  digitalWrite(CAN_BUS_TX_HOST_SIDE,1);
  digitalWrite(CAN_BUS_TX_CLIENT_SIDE,1);
  digitalWrite(LED_PIN,0);

  digitalWrite(CAN_BUS_SILENT_MODE,0);
  delay(500);

  RS485_HOST.write(0x55);

}


void loop() {
  // put your main code here, to run repeatedly:

  if(frame_done == 1)
  {
    frame_done =0;
    printf("new CAN frame:\r\n");
    printf("  Arbid: %ld\r\n", arbid);
    printf("  Msg: ");
    printf(" length:%u ",msg_len);
    for(int i = 0; i < msg_len; i++)
    {
        printf("%u,",message[i]);
        printf(" ");
    }
    printf("CRC: %lx\n", can_rx_crc);

  }

  if(Serial.available() > 0)
  {
      int dataIn = Serial.parseInt();

      if(dataIn % 2 || dataIn == 0)
      {
        targeted_rs485_id = dataIn;
        targeted_arbid = 0xFF;
        corruption_armed = true;
        printf("**********Now targeting ID %ld for fault injection**********\r\n", dataIn);

      }
      else
      {
        targeted_rs485_id = 0xFF;
        targeted_arbid = dataIn;
        corruption_armed = true;
        printf("**********Now targeting ARBID %ld for fault injection**********\r\n", dataIn);
      }


  }

  if(notify_fault_injected)
  {
    notify_fault_injected = false;
    printf("    ->Fault injected for ARBID %ld\r\n", targeted_arbid);
  }


  if(notify_fault_detected)
  {
    notify_fault_detected = false;
    printf("    ->Fault detected by the receiving MCU\r\n");
  }

  if(notify_fault_corrected)
  {
    notify_fault_corrected = false;
    printf("    ->Fault corrected by retransmission\r\n");
  }


  /*if( RS485_HOST.available() != 0)
  {
  }*/
  handle_rs485();
  //delay(500);
}


void handle_rs485(void)
{
  static state_rs485_receive_t current_state = SYNC;
  static bool rs485_client_is_receiver = true;
  static uint32_t msg_size =0;
  static uint32_t current_data_index = 0;
  static uint8_t current_id = 0;
  uint8_t byte_to_send;
  
  int dataAvaliable = (rs485_client_is_receiver)?  RS485_HOST.available():RS485_CLIENT.available()  ;
  if(dataAvaliable)
  {
    digitalWrite(LED_PIN,1);
     
    uint8_t byte =  (rs485_client_is_receiver)?  RS485_HOST.read():RS485_CLIENT.read();

    if(targeted_rs485_id == current_id && current_state == DATA)
    {
      byte_to_send = byte ^ 0b100;
    }
    else
    {
      byte_to_send = byte;
    }

    if(rs485_client_is_receiver){
      RS485_CLIENT.write(byte_to_send);
    }else{
      RS485_HOST.write(byte_to_send);
    }

    switch(current_state){
      case SYNC:
        printf("STATE SYNC\n");
        current_state = (byte==RS485_START_BYTE)?ID:SYNC;
      break;

      case ID:
        printf("STATE ID\n");
        current_id = byte;
        current_state = SIZE;
      break;

      case SIZE:
        printf("STATE SIZE\n");
        msg_size = byte;
        current_data_index=0;

        if(msg_size != 0)
        {
          current_state = DATA;
        }
        else
        {
          rs485_client_is_receiver = !rs485_client_is_receiver;
          current_state = SYNC;
        }
      break;

      case DATA:
        printf("STATE DATA\n");
        current_data_index++;

        if(current_data_index == msg_size)
        {
          rs485_client_is_receiver = !rs485_client_is_receiver;
          current_state = SYNC;
          printf("going back to sync\n");
        }

      break;

      default:
        printf("handle_rs485: unexpected state");
    };

    printf("Data: %u\n", byte);

  }

}

//This function get called by the interupt on the RX pin to detect
//the start of the CAN frame, this then start the timer to sample
//the whole frame
static void can_start_of_frame_detecte_handle(void)
{

    arbid = 0;
    can_rx_crc = 0;
    msg_byte = 0;
    extended_arbid = 0;
    msg_len = 0;
    tx_bit_count = 0;
    same_bits_count = 0;
    send_to_emmiter(1,client_is_receiver);
    send_to_receiver(1,client_is_receiver);

    t1.trigger(CAN_INITIAL_DELAY);
    //digitalWriteFast(CAN_BUS_TX_CLIENT_SIDE,digitalReadFast(CAN_BUS_RX_HOST_SIDE));

    //digitalWriteFast(0, !digitalReadFast(0));    /* Check to see if this is a stuff bit */
    detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE));
    detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE));

    /*if(client_is_receiver){
      detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE));
    }else{
      detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE));
    }*/
}



static void send_to_emmiter(int bit_sent, bool client_is_receiver)
{
  if(client_is_receiver){
    digitalWriteFast(CAN_BUS_TX_HOST_SIDE,bit_sent);
  }else {
    digitalWriteFast(CAN_BUS_TX_CLIENT_SIDE,bit_sent);
  }
}

static void send_to_receiver(int bit_sent, bool client_is_receiver)
{
  if(client_is_receiver){
    digitalWriteFast(CAN_BUS_TX_CLIENT_SIDE,bit_sent);
  }else {
    digitalWriteFast(CAN_BUS_TX_HOST_SIDE,bit_sent);
  }
}

static int read_emmiter(bool client_is_receiver)
{
  if(client_is_receiver){
    return digitalReadFast(CAN_BUS_RX_HOST_SIDE);
  }else {
    return digitalReadFast(CAN_BUS_RX_CLIENT_SIDE);
  }
}

static int read_receiver(bool client_is_receiver)
{
  if(client_is_receiver){
    return digitalReadFast(CAN_BUS_RX_CLIENT_SIDE);
  }else {
    return digitalReadFast(CAN_BUS_RX_HOST_SIDE);
  }
}

static void exit_error_state(void)
{
  if(client_is_receiver){
    detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE));
  }else{
    detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE));
  }

  send_to_emmiter(1,client_is_receiver);
  attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );
  attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );

  /*if(client_is_receiver)
  {
    attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
  }*/
}

static void enter_error_state(void)
{
  send_to_emmiter(0,client_is_receiver);
  notify_fault_detected = true;
  if(client_is_receiver)
  {
    attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), exit_error_state, RISING );
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), exit_error_state, RISING );
  }
}

/* Interrupt callback for sampling a CAN message. Uses the 
 * following global variables
 *
 * uint16_t bits_read = 0; 
 * uint8_t  last_bit = 0; 
 * uint8_t  same_bits_count = 0; 
 * uint32_t arbid; 
 * uint16_t msg_byte = 0; 
 * uint8_t  message[8]; 
 * uint8_t  extended_arbid = 0;
 * uint8_t  msg_len = 0;
 */
static void sample_callback(void)
{
    static int nb_bit_first_byte;
    static bool corrupt_next_bit = false;
    static bool current_message_corrupted = false;
    static int nb_message_since_last_retranmission =0;
    uint16_t bit_read = 0;
    bool currently_in_error_state = false;
    bit_read = read_emmiter(client_is_receiver);

    if(!corrupt_next_bit)
    {
      send_to_receiver(bit_read,client_is_receiver);
    }else{
      send_to_receiver(!bit_read,client_is_receiver);
      corrupt_next_bit = false;
      current_message_corrupted = false;
    }

    //digitalWriteFast(CAN_BUS_TX_CLIENT_SIDE,bit_read);

    /* Check to see if this is a stuff bit */
    if (same_bits_count >= 5) /* This is a stuff bit or an error frame */
    {
        if(bit_read == last_bit) // If we're reading the same bit a 6th time, this is an error frame
        {
            bits_read = 255; // This will effectively end the processing for this message and we will start over with the next one
        }
        else // Different bit level is for a stuff bit
        {
            // But don't forget that stuff bits count for the 5 same levels in a row
            same_bits_count = 1;
            last_bit = bit_read;
        }
    }
        
    else /* Not a stuff bit */
    {
        bits_read++;

        /* Track consecutive bits */
        if(bit_read == last_bit)
        {
            same_bits_count++;
        }
        else
            same_bits_count = 1;
        last_bit = bit_read;

        /* Populate the correct item based on position */
        // bit 1 is SOF, ignore it
        if(bits_read >= 2 && bits_read <= 12)
        {
            nb_bit_first_byte = 0;
            arbid <<= 1;
            arbid |= bit_read;
        }

        // bit 13 remote transmission request
        else if(bits_read == 14)
            extended_arbid = bit_read;

        /* From here, message length is based on whether or not we have an extended identifier. */
        if(extended_arbid == 0 && bits_read > 14)
        {
           if(bits_read == 15){
              if(arbid == KILL_SWITCH_ID || arbid == INTERFACE_ID)
                digitalWriteFast(0, !digitalReadFast(0));
            }
            // bit 15 is reserved
            if(bits_read >= 16 && bits_read <= 19)
            {
                msg_len <<= 1;
                msg_len |= bit_read;
            }
            else if(bits_read >= 20 && bits_read < (20 + (8 * msg_len)))
            {
                message[msg_byte] <<= 1;
                message[msg_byte] |= bit_read;

                if(nb_bit_first_byte == 1 && arbid == targeted_arbid)
                {
                  if(corruption_armed){
                    corrupt_next_bit = true;
                    corruption_armed = false;
                    notify_fault_injected = true;
                    next_message_retransmission = true;
                  }else if(next_message_retransmission){

                    nb_message_since_last_retranmission++;

                    if(nb_message_since_last_retranmission == 10)
                    {
                      nb_message_since_last_retranmission = 0;
                      corruption_armed = true;
                      notify_fault_corrected = true;
                      next_message_retransmission = false;
                    }

                  }

                }
                else if(nb_bit_first_byte == 3)
                {
                }
                nb_bit_first_byte++;
                if(bits_read - 20 - (msg_byte * 8) >= 7)
                    msg_byte++;
            }
            else if(bits_read >= (20 + (8 * msg_len)) && bits_read < (35 + (8 * msg_len)))
            {
                can_rx_crc <<= 1;
                can_rx_crc |= bit_read;
                if(bits_read < (34 + (8 * msg_len)-1))
                {
                  //corrupt_next_bit = true;
                  //corruption_armed = false;
                }
                if(bits_read == (34 + (8 * msg_len))) // Last bit of the CRC
                {
                    /* There is no bit stuffing for the CRC delimiter, ACK slot, or ACK delimiter */
                    same_bits_count = 0;
                }
            }
            else if(bits_read == (35 + (8 * msg_len))) // This is the CRC Delimiter
            {
            }
            else if(bits_read == (36 + (8 * msg_len))) // This is the ACK slot
            {
              //send_to_receiver(0,client_is_receiver);
              send_to_emmiter(0,client_is_receiver);

              //digitalWriteFast(CAN_BUS_TX_HOST_SIDE, 0);
              //digitalWriteFast(CAN_BUS_TX_CLIENT_SIDE,0);
            }
             else if(bits_read == (37 + (8 * msg_len))) // This is the ACK Delimiter
             {
              send_to_receiver(1,client_is_receiver);
              send_to_emmiter(1,client_is_receiver);
                //digitalWriteFast(CAN_BUS_TX_HOST_SIDE, 1);
                //digitalWriteFast(CAN_BUS_TX_CLIENT_SIDE,1);
             }
            else if((bits_read >= (38 + (8 * msg_len))) && (bits_read < (45 + (8 * msg_len)))) // End Of Frame bits
            {
              if(read_receiver(client_is_receiver) == LOW)
              {
                currently_in_error_state = true;
                enter_error_state();
                bits_read = 255; // This will effectively end the processing for this message and we will start over with the next one
              }
            }
            else if((bits_read >= (45 + (8 * msg_len))) && (bits_read < (48 + (8 * msg_len)))) // Inter Frame Segment
            {
              if(read_receiver(client_is_receiver) == LOW)
              {
                currently_in_error_state = true;
                enter_error_state();
                bits_read = 255; // This will effectively end the processing for this message and we will start over with the next one
              }

            }
        }
        else if (bits_read > 14)
        {
            if(bits_read >= 15 && bits_read <= 32)
            {
                arbid <<= 1;
                arbid |= bit_read;
            }
            // bit 33 is remote transmission request
            // bits 34 and 35 are reserved
            else if(bits_read >= 36 && bits_read <= 39)
            {
                msg_len <<= 1;
                msg_len |= bit_read;
            }
            else if(bits_read >= 40 && bits_read < (40 + (8 * msg_len)))
            {
                message[msg_byte] <<= 1;
                message[msg_byte] |= bit_read;

                if(bits_read - 40 - (msg_byte * 8) >= 7)
                    msg_byte++;
            }
            else if(bits_read >= (40 + (8 * msg_len)) && bits_read < (55 + (8 * msg_len)))
            {
                can_rx_crc <<= 1;
                can_rx_crc |= bit_read;


                if(bits_read == (54 + (8 * msg_len))) // Last bit of the CRC
                {
                    /* There is no bit stuffing for the CRC delimiter, ACK slot, or ACK delimiter */
                    same_bits_count = 0;
                }
            }
            else if(bits_read == (55 + (8 * msg_len))) // This is the CRC Delimiter
            {

            }
            else if(bits_read == (56 + (8 * msg_len))) // This is the ACK slot
            {

            }
            // else if(bits_read == (57 + (8 * msg_len))) // This is the ACK Delimiter
            // else if((bits_read >= (58 + (8 * msg_len))) && (bits_read < (65 + (8 * msg_len)))) // End Of Frame bits
            // else if((bits_read >= (65 + (8 * msg_len))) && (bits_read < (68 + (8 * msg_len)))) // Inter Frame Segment TODO: Not presently accounting for overload frames
        }
    }
    /* Currently skipping the EOF Header */
    //    if((extended_arbid == 0 && bits_read >= (20 + 18 + (8 * msg_len))) ||
    if((extended_arbid == 0 && bits_read >= (48 + (8 * msg_len))) ||
       (extended_arbid == 1 && bits_read >= (40 + 18 + (8 * msg_len))))
    {
        // Enable the external interrupt on the RX pin

        same_bits_count = 0;
        bits_read = 0;
        last_bit = 0;
        //digitalWriteFast(0, !digitalReadFast(0));    /* Check to see if this is a stuff bit */
        if(!currently_in_error_state)
        {
          frame_done = 1;
          frames_seen++;
          attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
          attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );

          if(client_is_receiver){

            if(arbid == KILL_SWITCH_ID && !current_message_corrupted){
              //attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
              client_is_receiver = false;
            }else{
              //attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );
              client_is_receiver = true;
            }

          }else{

            if(arbid == INTERFACE_ID && !current_message_corrupted){
              //attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );
              client_is_receiver = true;
            }else{
              //attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
              client_is_receiver = false;
            }
          }
        }

        current_message_corrupted = false;
        
        if(end_of_frame_callback != NULL)
            end_of_frame_callback();
    }
    else
    {
      t1.trigger(CAN_DELAY);
    }
    //digitalWriteFast(0, !digitalReadFast(0));    /* Check to see if this is a stuff bit */
}



