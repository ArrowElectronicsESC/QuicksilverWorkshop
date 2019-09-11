/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 * Azure HTTPS Client Application
 *
 * This application snippet demonstrates how send a message to Azure IoT hub
 *
 * Application Instructions
 * 1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *    in the wifi_config_dct.h header file to match your Wi-Fi access point
 * 2. Modify the HUBNAME, DEVICENAME, and SASTOKEN fields with credentials
 * 	  from your Azure IoT Hub.
 * 3. Modify the MESSAGE field is desired.
 * 4. Connect a PC terminal to the serial port of the WICED Eval board,
 *    then build and download the application as described in the WICED
 *    Quick Start Guide
 *
 * After the download completes, the application :
 *  - Connects to the Wi-Fi network specified
 *  - Resolves the Azure hub IP address using a DNS lookup
 *  - Sends HTTP request to Azure hub
 *
 */

#include <stdlib.h>
#include "wiced.h"
#include "wiced_tls.h"
#include "http_client.h"
#include "JSON.h"
#include "quicksilver.h"
//#include <debug.h>
#include "wiced_framework.h"
#include "ccan_json.h"

#define HUBNAME "<YOUR HUBNAME>"
#define DEVICENAME "<YOUR DEVICENAME>"
#define SASTOKEN "<YOUR SASTOKEN>"


#define URI1 "/devices/"DEVICENAME"/messages/events?api-version=2016-02-03"
#define CONTENT_TYPE "application/json;charset=utf-8"
#define SERVER_HOST   HUBNAME  ".azure-devices.net"
#define HTTP_HEADER_AUTHORIZATION "Authorization: "

#define SERVER_PORT        ( 443 )
#define DNS_TIMEOUT_MS     ( 10000 )
#define CONNECT_TIMEOUT_MS ( 3000 )
#define TOTAL_REQUESTS     ( 2 )


/******************************************************
 *                      Macros
 ******************************************************/
#define DELAY_MS(x)             wiced_rtos_delay_milliseconds(x)
#define VERIFY_SUCCESS(x)       if(x != WICED_SUCCESS) {wiced_framework_reboot();}

/******************************************************
 *                    Constants
 ******************************************************/
#define RGB_CLOCK WICED_GPIO_10
#define RGB_DATA WICED_GPIO_8
#define BUFFER_LENGTH     (2048)
#define MAX_LINE_LENGTH  (128)
#define MAX_HISTORY_LENGTH (20)
#define MAX_NUM_COMMAND_TABLE  (8)
#define NUM_I2C_MESSAGE_RETRIES   (3)
#define PING_TIMEOUT_MS          2000

/******************************************************
 *                   Enumerations
 ******************************************************/
enum LED_COLOR_VALUES
{
    LED_COLOR_OFF = 0x00,
    LED_COLOR_MIN = 0x01,
    LED_COLOR_MAX = 0xFF
};

enum LED_BRIGHTNESS_VALUES
{
    LED_BRIGHTNESS_OFF = 0x00,
    LED_BRIGHTNESS_MIN = 0x01,
    LED_BRIGHTNESS_MAX = 0x1F
};

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;

typedef struct {
    char * cmd;
    int (*handler)(const char *data);
}command_handler_t;

/******************************************************
 *                    Structures
 ******************************************************/
wiced_result_t temperature_init( void );
wiced_result_t accelerometer_init( void );
wiced_result_t rgb_init( void );
int cmd_handler_setLED(const char *data);
int cmd_handler_updateLED(const char *data);
int cmd_handler_update(const char *data);

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void  event_handler( http_client_t* client, http_event_t event, http_response_t* response );
static void  print_data   ( char* data, uint32_t length );
static void  print_content( char* data, uint32_t length );
static void  print_header ( http_header_field_t* header, uint32_t number_of_fields );
static wiced_result_t parse_http_response_info(wiced_json_object_t * json_object );

static wiced_i2c_device_t i2c_device_temperature =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = HTS221_I2C_ADDRESS,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};

static wiced_i2c_device_t i2c_device_accelerometer =
{
        .port = WICED_I2C_2,  //I2C_1
        .address = LIS2DH12_I2C_ADD_H,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE,
};


/******************************************************
 *               Variable Definitions
 ******************************************************/

static const char azure_root_ca_certificate[] =
        "-----BEGIN CERTIFICATE-----\r\n"
        "MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
        "RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
        "VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
        "DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
        "ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
        "VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
        "mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
        "IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
        "mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
        "XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
        "dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
        "jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
        "BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
        "DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
        "9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
        "jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
        "Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
        "ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
        "R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
        "-----END CERTIFICATE-----\r\n";

const int content_length_buffer_size = 8;
char content_length_buffer[8];

static quicksilver_data telemetryData;
static lis2dh12_ctx_t accel_ctx;
static hts221_ctx_t hts_ctx;
static apa102_ctx_t rgb_ctx;
static axis3bit16_t data_raw_acceleration;
static float acceleration_mg[3];
static axis1bit16_t data_raw_humidity;
static axis1bit16_t data_raw_temperature;
static float humidity_perc;
static float temperature_degC;
static uint8_t whoamI;
static axis1bit16_t coeff;
static lin_t lin_hum;
static lin_t lin_temp;
static char* json_message;


/******************************************************
 *               CTX Interface Function Definitions
 ******************************************************/

/* Function for performing an I2C write */
int32_t i2c_write(void * handle, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t write_buffer[length + 1];
    write_buffer[0] = (reg | 0x80);
    memcpy(&write_buffer[1], buffer, length);

    wiced_i2c_write( handle, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, write_buffer, sizeof(write_buffer) );

    return 0;
}

/* Function for performing an I2C read */
int32_t i2c_read(void * handle, uint8_t reg, uint8_t* buffer, uint16_t length)
{
    uint8_t tx_buffer [1] = {(reg | 0x80)};

    wiced_i2c_write( handle, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, tx_buffer, 1 );
    wiced_i2c_read( handle, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, buffer, length );

    return 0;
}

/* Function for setting a pin in the RGB driver */
int32_t rgb_pin_set(void * dev_ctx, uint32_t pin)
{
    wiced_gpio_output_high(pin);

    return 0;
}

/* Function for clearing a pin in the RGB driver */
int32_t rgb_pin_clear(void * dev_ctx, uint32_t pin)
{
    wiced_gpio_output_low(pin);

    return 0;
}

/* Function for performing an RTOS delay in the RGB driver */
int32_t rgb_delay_ms(void * dev_ctx, uint32_t milliseconds)
{
    DELAY_MS(milliseconds);

    return 0;
}

/******************************************************
 *               Helper Function Definitions
 ******************************************************/

/* Function used to apply coefficient */
float linear_interpolation(lin_t *lin, int16_t x)
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}



/******************************************************
 *               Application Function Definitions
 ******************************************************/

/* Function for probing sensors on the I2C bus. */
wiced_result_t i2c_sensor_probe(void)
{
    /* Probe I2C bus for accelerometer */
    if( wiced_i2c_probe_device( &i2c_device_accelerometer, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    /* Probe I2C bus for temperature sensor */
    if( wiced_i2c_probe_device( &i2c_device_temperature, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Function for initializing the I2C bus */
wiced_result_t i2c_init(void)
{
    /* Initialize I2C for the accelerometer */
    if ( wiced_i2c_init( &i2c_device_accelerometer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Initialize I2C for the temperature sensor */
    if ( wiced_i2c_init( &i2c_device_temperature ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Probe the I2C bus for the sensors */
    if ( i2c_sensor_probe() != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Initialize GPIO */
wiced_result_t gpio_init(void)
{
    if(wiced_gpio_init( WICED_RGB_CLOCK, OUTPUT_PUSH_PULL ) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    if(wiced_gpio_init( WICED_RGB_DATA, OUTPUT_PUSH_PULL ) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/* Function for initializing the temperature sensor */
wiced_result_t temperature_init( void )
{
    /* Initialize mems driver interface */
    hts_ctx.write_reg = i2c_write;
    hts_ctx.read_reg = i2c_read;
    hts_ctx.handle = &i2c_device_temperature;

    /* Read the device ID to verify communications to the sensor */
    hts221_device_id_get(&hts_ctx, &whoamI);
    if( whoamI != HTS221_ID )
    {
        WPRINT_APP_INFO(("Failed to read WHOAMI from temperature device; addr 0x%x\r\n", i2c_device_temperature.address));
        return WICED_ERROR;
    }

    /* Read humidity calibration coefficient */
    hts221_hum_adc_point_0_get(&hts_ctx, coeff.u8bit);
    lin_hum.x0 = (float)coeff.i16bit;
    hts221_hum_rh_point_0_get(&hts_ctx, coeff.u8bit);
    lin_hum.y0 = (float)coeff.u8bit[0];
    hts221_hum_adc_point_1_get(&hts_ctx, coeff.u8bit);
    lin_hum.x1 = (float)coeff.i16bit;
    hts221_hum_rh_point_1_get(&hts_ctx, coeff.u8bit);
    lin_hum.y1 = (float)coeff.u8bit[0];

    /* Read temperature calibration coefficient */
    hts221_temp_adc_point_0_get(&hts_ctx, coeff.u8bit);
    lin_temp.x0 = (float)coeff.i16bit;
    hts221_temp_deg_point_0_get(&hts_ctx, coeff.u8bit);
    lin_temp.y0 = (float)coeff.u8bit[0];
    hts221_temp_adc_point_1_get(&hts_ctx, coeff.u8bit);
    lin_temp.x1 = (float)coeff.i16bit;
    hts221_temp_deg_point_1_get(&hts_ctx, coeff.u8bit);
    lin_temp.y1 = (float)coeff.u8bit[0];

    /* Enable Block Data Update */
    hts221_block_data_update_set(&hts_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    hts221_data_rate_set(&hts_ctx, HTS221_ODR_12Hz5);

    /* Device power on */
    hts221_power_on_set(&hts_ctx, PROPERTY_ENABLE);

    return WICED_SUCCESS;
}

/* Function to getting temperature and humidity data from the sensor */
int temperature_get(int argc, char *argv[])
{
    hts221_reg_t reg;
    hts221_status_get(&hts_ctx, &reg.status_reg);

    /* Get Humidity data if new data available */
    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
        memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
        hts221_humidity_raw_get(&hts_ctx, data_raw_humidity.u8bit);
        humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
        if (humidity_perc < 0) humidity_perc = 0;
        if (humidity_perc > 100) humidity_perc = 100;
        telemetryData.humidity = humidity_perc;
    }

    /* Get temperature data if new data available */
    if (reg.status_reg.t_da)
    {
        /* Read temperature data */
        memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
        hts221_temperature_raw_get(&hts_ctx, data_raw_temperature.u8bit);
        temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
        telemetryData.temperature = temperature_degC;
    }

    return 0;
}

/* Function for initializing the accelerometer */
wiced_result_t accelerometer_init( void )
{
   /* Initialize mems driver interface */
    accel_ctx.write_reg = i2c_write;
    accel_ctx.read_reg = i2c_read;
    accel_ctx.handle = &i2c_device_accelerometer;

    /* Read the device ID to verify communications to the sensor */
    lis2dh12_device_id_get(&accel_ctx, &whoamI);
    if(whoamI != LIS2DH12_ID)
    {
        WPRINT_APP_INFO(("Failed to read WHOAMI from accelerometer device; addr 0x%x\r\n", i2c_device_accelerometer.address));
    }

    /* Enable Block Data Update */
    lis2dh12_block_data_update_set(&accel_ctx, PROPERTY_ENABLE);

    /* Set the Data Rate */
    lis2dh12_data_rate_set(&accel_ctx, LIS2DH12_ODR_400Hz);

    /* Set the scale */
    lis2dh12_full_scale_set(&accel_ctx, LIS2DH12_4g);

    /* Set-up temperature measurements */
    lis2dh12_temperature_meas_set(&accel_ctx, LIS2DH12_TEMP_ENABLE);

    /* Set normal mode */
    lis2dh12_operating_mode_set(&accel_ctx ,LIS2DH12_HR_12bit);

    return WICED_SUCCESS;
}

/* Function for getting accelerometer data from the sensor */
int accelerometer_get(int argc, char *argv[])
{

    lis2dh12_reg_t reg;
    lis2dh12_status_get(&accel_ctx, &reg.status_reg);

    /* Get new accelerometer data if new data available */
    if( reg.status_reg.zyxda )
    {

        memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
        lis2dh12_acceleration_raw_get(&accel_ctx, data_raw_acceleration.u8bit);
        acceleration_mg[0] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[0] );
        acceleration_mg[1] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[1] );
        acceleration_mg[2] = LIS2DH12_FROM_FS_4g_HR_TO_mg( data_raw_acceleration.i16bit[2] );

        telemetryData.accelerometer.x = acceleration_mg[0];
        telemetryData.accelerometer.y = acceleration_mg[1];
        telemetryData.accelerometer.z = acceleration_mg[2];
    }

    return 0;
}

/* Initializes RGB LED driver */
wiced_result_t rgb_init( void )
{
    rgb_ctx.clk_in_pin = WICED_RGB_CLOCK;
    rgb_ctx.data_in_pin = WICED_RGB_DATA;
    rgb_ctx.pin_set = rgb_pin_set;
    rgb_ctx.pin_clear = rgb_pin_clear;
    rgb_ctx.delay_ms = rgb_delay_ms;

    apa102_init(&rgb_ctx);

    // Set the LED red during initialization
    apa102_led_brightness_set(&rgb_ctx, LED_BRIGHTNESS_MIN);
    apa102_led_red_set(&rgb_ctx, LED_COLOR_MAX);

    return WICED_SUCCESS;
}

/* Function for getting the RGB LED color data. */
int rgb_color_get(int argc, char *argv[])
{
    apa102_color_get(&telemetryData.led);

    return WICED_SUCCESS;
}

/* Function for updating all Quicksilver sensor data. */
int update_sensor_data(void * data)
{
    accelerometer_get(0, NULL);
    temperature_get(0, NULL);
    rgb_color_get(0, NULL);

    return WICED_SUCCESS;
}


/* Function for initializing the Quicksilver board and on-board hardware */
wiced_result_t quicksilver_init(void)
{
    /* Initialize I2C*/
    if(i2c_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Initialize GPIO*/
    if(gpio_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Initialize the RGB */
    if(rgb_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* probe for temperature device */
    if(temperature_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* probe for accelerometer device */
    if(accelerometer_init() != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}



/******************************************************
 *               Function Definitions
 ******************************************************/
static http_client_t  client;
static http_request_t requests[TOTAL_REQUESTS];
static http_client_configuration_info_t client_configuration;

static const char* request_uris[] =
{
    [0] = URI1,
};

// Updating the message with current telemetry data, formated as JSON
static void update_msg(void)
{
    update_sensor_data(&telemetryData);

    JsonNode * node = json_mkobject();
    json_append_member(node, "DeviceID", json_mkstring(DEVICENAME));
    json_append_member(node, "Temperature", json_mknumber(telemetryData.temperature));
    json_append_member(node, "Humidity", json_mknumber(telemetryData.humidity));
    json_append_member(node, "AccelerationX", json_mknumber((double)telemetryData.accelerometer.x));
    json_append_member(node, "AccelerationY", json_mknumber((double)telemetryData.accelerometer.y));
    json_append_member(node, "AccelerationZ", json_mknumber((double)telemetryData.accelerometer.z));
    json_message = json_stringify(node, " ");
//    WPRINT_APP_INFO(("JSON Message: %s", json_message));
}


wiced_result_t RECONNECT = WICED_FALSE;

void application_start( )
{
    wiced_ip_address_t ip_address;
    wiced_result_t     result;
    http_header_field_t header[5];

    wiced_init( );

    quicksilver_init();

    wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

    WPRINT_APP_INFO( ( "Resolving IP address of %s\n", SERVER_HOST ) );
    result = wiced_hostname_lookup( SERVER_HOST, &ip_address, DNS_TIMEOUT_MS , WICED_STA_INTERFACE);
    if(result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Error: could not resolve\nCheck your HUBNAME or Press reset on board\n"));
        return;
    }
    else WPRINT_APP_INFO( ( "%s is at %u.%u.%u.%u\n", SERVER_HOST,
                             (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 24),
                             (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 16),
                             (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 8),
                             (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 0) ) );

    /* Initialize the root CA certificate */
    result = wiced_tls_init_root_ca_certificates( azure_root_ca_certificate, strlen( azure_root_ca_certificate ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Error: Root CA certificate failed to initialize: %u\n", result) );
        return;
    }

    http_client_init( &client, WICED_STA_INTERFACE, event_handler, NULL );
       WPRINT_APP_INFO( ( "Connecting to %s\n", SERVER_HOST ) );


    /* configure HTTP client parameters */
        client_configuration.flag = HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME | HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN;
        client_configuration.server_name = (uint8_t*)SERVER_HOST;
        client_configuration.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;
        http_client_configure(&client, &client_configuration);

        result = http_client_connect( &client, (const wiced_ip_address_t*)&ip_address, SERVER_PORT, HTTP_USE_TLS, CONNECT_TIMEOUT_MS );
        if(result == WICED_SUCCESS) WPRINT_APP_INFO( ( "Connected\n" ) );

        // Continuous send data to Azure in HTTPS format every 1 second(s)
        while(1)
        {
            if(RECONNECT)
            {
                result = http_client_connect( &client, (const wiced_ip_address_t*)&ip_address, SERVER_PORT, HTTP_USE_TLS, CONNECT_TIMEOUT_MS );
                if(result == WICED_SUCCESS)
                {
                    WPRINT_APP_INFO( ( "Connected\n" ) );
                    RECONNECT = WICED_FALSE;
                }
            }

            wiced_JSON_parser_register_callback(parse_http_response_info);  // Parse HTTP response info

            update_msg();                                                   // Update message with latest telemetry data

            // Populate HTTP packet with header and data
            header[0].field        = HTTP_HEADER_HOST;
            header[0].field_length = sizeof( HTTP_HEADER_HOST ) - 1;
            header[0].value        = SERVER_HOST;
            header[0].value_length = sizeof( SERVER_HOST ) - 1;

            header[1].field        = HTTP_HEADER_AUTHORIZATION;
            header[1].field_length = sizeof( HTTP_HEADER_AUTHORIZATION ) - 1;
            header[1].value        = SASTOKEN;
            header[1].value_length = sizeof( SASTOKEN ) - 1;

            header[2].field        = HTTP_HEADER_CONTENT_TYPE;
            header[2].field_length = sizeof( HTTP_HEADER_CONTENT_TYPE ) - 1;
            header[2].value        = CONTENT_TYPE;
            header[2].value_length = sizeof( CONTENT_TYPE ) - 1;

            snprintf(content_length_buffer,content_length_buffer_size,"%d", (short)strlen(json_message));
            header[3].field        = HTTP_HEADER_CONTENT_LENGTH;
            header[3].field_length = sizeof( HTTP_HEADER_CONTENT_LENGTH ) - 1;
            header[3].value        = content_length_buffer;
            header[3].value_length = strlen( content_length_buffer );

            header[4].field        = HTTP_CLRF;
            header[4].field_length = sizeof( HTTP_CLRF ) - 1;
            header[4].value        = json_message;
            header[4].value_length = strlen( json_message );

            http_request_init( &requests[0], &client, HTTP_POST, request_uris[0], HTTP_1_1 );
            http_request_write_header( &requests[0], &header[0], 5 );
            http_request_write_end_header( &requests[0] );
            http_request_flush( &requests[0] );

            DELAY_MS(1000);     // Delay 1 second(s)
        }

}

static void event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    switch( event )
    {
        case HTTP_DISCONNECTED:
        {
            WPRINT_APP_INFO(( "Disconnected from %s\n", SERVER_HOST ));
            http_request_deinit( &requests[0] );
            http_request_deinit( &requests[1] );
            http_client_disconnect(client);

            RECONNECT = WICED_TRUE;
            break;
        }

        case HTTP_DATA_RECEIVED:
        {
            if ( response->request == &requests[0] )
            {
                /* Response to first request. Simply print the result */
                WPRINT_APP_INFO( ( "\nRecieved response for request #1. Content received:\n" ) );

                /* print only HTTP header */
                if(response->response_hdr != NULL)
                {
                    WPRINT_APP_INFO( ( "\n HTTP Header Information for response1 : \n" ) );
                    print_content( (char*) response->response_hdr, response->response_hdr_length );
                }

                /* print payload information comes as HTTP response body */
                WPRINT_APP_INFO( ( "\n Payload Information for response1 : \n" ) );
                print_content( (char*) response->payload, response->payload_data_length );
                if(wiced_JSON_parser( (const char*)response->payload , response->payload_data_length ) != WICED_SUCCESS)
                {
                    WPRINT_APP_ERROR( ( "\n JSON parsing Error: \n" ) );
                }

                if(response->remaining_length == 0)
                {
                   WPRINT_APP_INFO( ( "Received total payload data for response1 \n" ) );
                }
            }
            else if ( response->request == &requests[1] )
            {
                /* Response to 2nd request. Simply print the result */
                WPRINT_APP_INFO( ( "\nRecieved response for request #2. Content received:\n" ) );

                /* Response to second request. Parse header for "Date" and "Content-Length" */
                http_header_field_t header_fields[2];
                uint32_t size = sizeof( header_fields ) / sizeof(http_header_field_t);

                /* only process HTTP header when response contains it */
                if(response->response_hdr != NULL)
                {
                    WPRINT_APP_INFO( ( "\n HTTP Header Information for response2 : \n" ) );
                    print_content( (char*) response->response_hdr, response->response_hdr_length );

                    header_fields[ 0 ].field        = HTTP_HEADER_DATE;
                    header_fields[ 0 ].field_length = sizeof( HTTP_HEADER_DATE ) - 1;
                    header_fields[ 0 ].value        = NULL;
                    header_fields[ 0 ].value_length = 0;
                    header_fields[ 1 ].field        = HTTP_HEADER_CONTENT_LENGTH;
                    header_fields[ 1 ].field_length = sizeof( HTTP_HEADER_CONTENT_LENGTH ) - 1;
                    header_fields[ 1 ].value        = NULL;
                    header_fields[ 1 ].value_length = 0;

                    if ( http_parse_header( response->response_hdr, response->response_hdr_length, header_fields, size ) == WICED_SUCCESS )
                    {
                        WPRINT_APP_INFO( ( "\nParsing response of request #2 for \"Date\" and \"Content-Length\". Fields found:\n" ) );
                        print_header( header_fields, size );
                    }
                }

                /* Print payload information that comes as response body */
                WPRINT_APP_INFO( ( "Payload Information for response2 : \n" ) );
                print_content( (char*) response->payload, response->payload_data_length );

                if(response->remaining_length == 0)
                {
                    WPRINT_APP_INFO( ( "Received total payload data for response2 \n" ) );
                }
            }
        break;
        }
        default:
        break;
    }
}

static void print_data( char* data, uint32_t length )
{
    uint32_t a;

    for ( a = 0; a < length; a++ )
    {
        WPRINT_APP_INFO( ( "%c", data[a] ) );
    }
}

static void print_content( char* data, uint32_t length )
{
    WPRINT_APP_INFO(( "==============================================\n" ));
    print_data( (char*)data, length );
    WPRINT_APP_INFO(( "\n==============================================\n" ));
}

static void print_header( http_header_field_t* header_fields, uint32_t number_of_fields )
{
    uint32_t a;

    WPRINT_APP_INFO(( "==============================================\n" ));
    for ( a = 0; a < number_of_fields; a++ )
    {
        print_data( header_fields[a].field, header_fields[a].field_length );
//        WPRINT_APP_INFO(( " : " ));
        print_data( header_fields[a].value, header_fields[a].value_length );
        WPRINT_APP_INFO(( "\n" ));
    }
    WPRINT_APP_INFO(( "==============================================\n" ));
}

static wiced_result_t parse_http_response_info(wiced_json_object_t * json_object )
{
    if(strncmp(json_object->object_string, "url", sizeof("url")-1) == 0)
    {
        WPRINT_APP_INFO (("Requested URL : %.*s\n",json_object->value_length, json_object->value));
    }

    return WICED_SUCCESS;
}
