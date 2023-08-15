#include "TeensyTimerTool.h"
#include "hamming_toolkit.h"
#include <cmath>
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
#define CAN_INITIAL_DELAY (2800ns)
#define CAN_DELAY (8138ns)

//pin definition
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

//rs485 protocol definition
#define RS485_START_BYTE 0x55
#define BAUDRATE_RS485 115200

//timer for can sampling
OneShotTimer t1(GPT1);
OneShotTimer t2(GPT2);

typedef enum{
  HEADER_HAMMING,
  DATA_HAMMING
} state_rs485_receive_hamming_t;



static void sample_callback(void);
static void can_start_of_frame_detecte_handle(void);
void setup() {

  //pin setup
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

  //rs485 setup
  RS485_CLIENT.begin(BAUDRATE_RS485);
  RS485_CLIENT.transmitterEnable(RS485_DRIVER_EN_CLIENT);
  RS485_HOST.begin(BAUDRATE_RS485);
  RS485_HOST.transmitterEnable(RS485_DRIVER_EN_HOST);
  digitalWrite(RS485_TERM_EN_HOST,1);
  digitalWrite(RS485_RECEIVER_EN_HOST,0);
  digitalWrite(RS485_TERM_EN_CLIENT,1);
  digitalWrite(RS485_RECEIVER_EN_CLIENT,0);

  pinMode(0,OUTPUT);

  //can receive attach interupt
  attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );
  attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
  
  t1.begin(sample_callback);
  Serial.begin(9600);

  //out rx in rececive state
  digitalWrite(CAN_BUS_TX_HOST_SIDE,1);
  digitalWrite(CAN_BUS_TX_CLIENT_SIDE,1);
  digitalWrite(LED_PIN,0);

  digitalWrite(CAN_BUS_SILENT_MODE,0);
  delay(500);

  RS485_HOST.write(0x55);

}


void loop() {

  //print information from the decoded CAN frame
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

  //listen for id of the messages to corrupt
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

  //print information about CAN bus corruption
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


  handle_rs485_hamming();
}

//decode and transfert 485 messagea, posibly inject error
void handle_rs485_hamming(void)
{
  static state_rs485_receive_hamming_t current_state = HEADER_HAMMING;
  static bool rs485_client_is_receiver = true;
  static uint32_t msg_size =0;
  static uint32_t current_data_index = 0;
  static uint8_t current_id = 0;
  static size_t current_header_position=0;
  uint8_t byte_to_send;

  const size_t HEADER_DECODED_SIZE = 3;
  const size_t HEADER_ENCODED_SIZE = 2*HEADER_DECODED_SIZE;
  const size_t HEADER_ENCODED_PACK_SIZE = (size_t) ceil(((double)HEADER_ENCODED_SIZE*7)/8);

  static uint8_t pack_header[HEADER_ENCODED_PACK_SIZE] = {0};
  static uint8_t unpack_header[HEADER_ENCODED_SIZE] = {0};
  static uint8_t deinterleaved_header[HEADER_ENCODED_SIZE] = {0};
  static uint8_t decoded_header[HEADER_DECODED_SIZE] = {0};


  int dataAvaliable = (rs485_client_is_receiver)?  RS485_HOST.available():RS485_CLIENT.available()  ;
  if(dataAvaliable)
  {
    digitalWrite(LED_PIN,1);
     
    uint8_t byte =  (rs485_client_is_receiver)?  RS485_HOST.read():RS485_CLIENT.read();

    //corrupt message if need be
    if(targeted_rs485_id == current_id && current_state == DATA_HAMMING && current_data_index == 0)
    {
      printf("Injecting error\n");
      byte_to_send = byte ^ 0b1100;
    }
    else
    {
      byte_to_send = byte;
    }

    //transfert data to the other side
    if(rs485_client_is_receiver){
      RS485_CLIENT.write(byte_to_send);
    }else{
      RS485_HOST.write(byte_to_send);
    }

    //state machine to decode RS485 protocole
    switch(current_state){
      case HEADER_HAMMING:
        printf("STATE SYNC\n");

        pack_header[current_header_position] = byte;
        current_header_position++;
        
        //once the header is fully received, decode it
        if(current_header_position == HEADER_ENCODED_PACK_SIZE)
        {
          HammingToolkit::unpack_7_bits_values(pack_header, HEADER_ENCODED_PACK_SIZE, unpack_header, HEADER_ENCODED_SIZE);
          HammingToolkit::deinterleaving_post_depack(deinterleaved_header, HEADER_ENCODED_SIZE, unpack_header);
          HammingToolkit::decode_hamming_74_message(deinterleaved_header, HEADER_ENCODED_SIZE, decoded_header, HEADER_DECODED_SIZE);

          current_data_index = 0;
          current_header_position = 0;

          current_id = decoded_header[1];

          //compute message size of the encoded message
          msg_size = (uint32_t) ceil(((double)decoded_header[2]*2*7)/8);
          printf("current_id:%u (targeted_rs485_id: %u)\n",current_id, targeted_rs485_id);
          printf("msg size = %d (%d)\n",msg_size,HEADER_ENCODED_PACK_SIZE);

          if(msg_size != 0)
          {
            current_state = DATA_HAMMING;
          }
          else
          {
            //is msg_size is 0, return back to wait for header
            rs485_client_is_receiver = !rs485_client_is_receiver;
            current_state = HEADER_HAMMING;
          }

          if(targeted_rs485_id == current_id)
            printf("id\n");

          if(current_state == DATA_HAMMING)
            printf("state\n");

          if(current_data_index == 0)
            printf("index\n");


          if(targeted_rs485_id == current_id && current_state == DATA_HAMMING && current_data_index == 0)
          {
            printf("ok\n");
          }
        }
      break;

      case DATA_HAMMING:
        printf("STATE DATA\n");
        current_data_index++;

        if(current_data_index == msg_size)
        {
          rs485_client_is_receiver = !rs485_client_is_receiver;
          current_state = HEADER_HAMMING;
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

    //reset values for can sampling
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
    detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE));
    detachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE));

}


//functions to send/read depending on who is the emmiter/receiver
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



//version modifié du code de bitbane (https://github.com/bitbane/CANT)
//sous licence BSD
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

    //bit corruption
    if(!corrupt_next_bit)
    {
      send_to_receiver(bit_read,client_is_receiver);
    }else{
      send_to_receiver(!bit_read,client_is_receiver);
      corrupt_next_bit = false;
      current_message_corrupted = false;
    }


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
              send_to_emmiter(0,client_is_receiver);
            }
             else if(bits_read == (37 + (8 * msg_len))) // This is the ACK Delimiter
             {
              send_to_receiver(1,client_is_receiver);
              send_to_emmiter(1,client_is_receiver);
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
        //end of the CAN frame, setup for next frame
        same_bits_count = 0;
        bits_read = 0;
        last_bit = 0;
        if(!currently_in_error_state)
        {
          frame_done = 1;
          frames_seen++;
          //wait for a new receive
          attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_CLIENT_SIDE), can_start_of_frame_detecte_handle, FALLING );
          attachInterrupt(digitalPinToInterrupt(CAN_BUS_RX_HOST_SIDE), can_start_of_frame_detecte_handle, FALLING );

          //switch who is the receiver
          if(client_is_receiver){
            if(arbid == KILL_SWITCH_ID && !current_message_corrupted){
              client_is_receiver = false;
            }else{
              client_is_receiver = true;
            }

          }else{
            if(arbid == INTERFACE_ID && !current_message_corrupted){
              client_is_receiver = true;
            }else{
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
      //frame is not ended, schedule next sample 
      t1.trigger(CAN_DELAY);
    }
}



