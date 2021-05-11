#include "mbed.h"
#include "math.h"
#include "stm32l475e_iot01_accelero.h"
#include "mbed_rpc.h"
#include "uLCD_4DGL.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "accelerometer_handler.h"
#include "magic_wand_model_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#ifndef CONFIG_H_
#define CONFIG_H_

// The number of labels (without negative)
#define label_num 3

struct Config {

  // This must be the same as seq_length in the src/model_train/config.py
  const int seq_length = 64;

  // The number of expected consecutive inferences for each gesture type.
  const int consecutiveInferenceThresholds[label_num] = {20, 10, 15};

  const char* output_message[label_num] = {
        "RING:\n\r"
        "          *       \n\r"
        "       *     *    \n\r"
        "     *         *  \n\r"
        "    *           * \n\r"
        "     *         *  \n\r"
        "       *     *    \n\r"
        "          *       \n\r",
        "SLOPE:\n\r"
        "        *        \n\r"
        "       *         \n\r"
        "      *          \n\r"
        "     *           \n\r"
        "    *            \n\r"
        "   *             \n\r"
        "  *              \n\r"
        " * * * * * * * * \n\r",
        "SHAKE:\n\r"
        "                 \n\r"
        "                 \n\r"
        "                 \n\r"
        "*****************\n\r"
        "                 \n\r"
        "                 \n\r"
        "                 \n\r"
        "                 \n\r"};
};

Config config;
#endif // CONFIG_H_

void gesture_UI(Arguments *in, Reply *Out);
void threshold_select(void);
void angle_det(Arguments *in, Reply *Out);
void thread_angle(void);
void stop_mode(Arguments *in, Reply *Out);

void publish_Threshold(MQTT::Client<MQTTNetwork, Countdown>* client);
void publish_tilt(MQTT::Client<MQTTNetwork, Countdown>* client, double angle);
void messageArrived(MQTT::MessageData& md);

int PredictGesture(float* output);
double calculate_angle(int16_t ref_XYZ[3], int16_t tilt_XYZ[3]);

DigitalOut led1(LED1);  // stand for gesture_UI mode
DigitalOut led2(LED2);  // stand for angle detection mode
DigitalOut led3(LED3);  // if 1, user can begin to tilt the board
DigitalIn confirm_select(USER_BUTTON);
BufferedSerial pc(USBTX, USBRX);
uLCD_4DGL uLCD(D1, D0, D2); // connection of uLCD
RPCFunction rpc_gesture_UI(&gesture_UI, "gesture_UI");
RPCFunction rpc_angle_det(&angle_det, "angle_det");
RPCFunction rpc_stop_mode(&stop_mode, "stop_mode");
WiFiInterface *wifi;
MQTT::Client<MQTTNetwork, Countdown> *pointer_client;   // for the use in publish_Threshold function
Thread tRPC_ges, tRPC_angle;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
int Threshold_angle = 30;
int num_tilt = 0;
bool stop = false;
int i = ((SIZE_X / 5) - 6) / 2 - 1;     // coordinate of cursor on ulCD
int j = ((SIZE_Y / 7) - 1) / 2 - 1;

double angle = 0;
// The gesture index of the prediction
int gesture_index;
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

const char* topic = "Mbed";
const char* topic1 = "Mbed/Threshold";
const char* topic2 = "Mbed/angle_detected";

int main()
{
    led1 = 0;   // initialize the three LED
    led2 = 0;
    led3 = 0;

    BSP_ACCELERO_Init();        // The accelerometer need to be initialized first.
    
    uLCD.background_color(WHITE);
    uLCD.cls();
    uLCD.textbackground_color(WHITE);
    uLCD.color(BLACK);
    uLCD.set_font(FONT_5X7);
    uLCD.locate(i,j);

    // -------------- connect to wifi and MQTT --------------------
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
        printf("ERROR: No WiFiInterface found.\r\n");
        return -1;
    }

    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        printf("\nConnection error: %d\r\n", ret);
        return -1;
    }

    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    pointer_client = &client;

    const char* host = "192.168.43.211";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
        printf("Connection error.\r\n");
        return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
        printf("Fail to connect MQTT\r\n");
        return -1;
    }
    if (client.subscribe(topic1, MQTT::QOS0, messageArrived) != 0){
        printf("Fail to subscribe for topic 1\r\n");
        return -1;
    }
    if (client.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
        printf("Fail to subscribe for topic 2\r\n");
        return -1;
    }
    int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }
    // ------------------ end of connection -----------------------

    char buf[256], outbuf[256]; // for RPC call
    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    while (true) {
        memset(buf, 0, 256);      // clear buffer
        for(int i=0; i<255; i++) {
            char recv = fgetc(devin);
            if (recv == '\r' || recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }

    printf("Ready to close MQTT Network......\n");
    if ((rc = client.unsubscribe(topic1)) != 0) {
        printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.unsubscribe(topic2)) != 0) {
        printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
        printf("Failed: rc from disconnect was %d\n", rc);
    }
    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
    return 0;
}

void gesture_UI(Arguments *in, Reply *Out) {
    Thread t1;
    led1 = 1;
    Threshold_angle = 30;

    t1.start(threshold_select);
    while (confirm_select == 1) {
        //uLCD.cls();
        uLCD.locate(i, j);
        uLCD.printf("%d", Threshold_angle);
    }
    t1.terminate();

    publish_Threshold(pointer_client);

    // ----------------- wait for command to jump out of this mode --------------------
    char Threshold_buf[256], Threshold_outbuf[256]; // for RPC call
    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    memset(Threshold_outbuf, 0, 256);
    while (1) {         // wait for Python to stop gesture UI mode.
        memset(Threshold_buf, 0, 256);      // clear buffer
        for(int i=0; i<255; i++) {
            char recv = fgetc(devin);
            if (recv == '\r' || recv == '\n') {
                printf("\r\n");
                break;
            }
            Threshold_buf[i] = fputc(recv, devout);
        }
        RPC::call(Threshold_buf, Threshold_outbuf);
        if (stop == true)
            break;
    }
    stop = false;
    // ---------------------------- end of waiting -------------------------------------

    uLCD.cls();
    led1 = 0;
    Out->putData("Jump out of gesture UI mode.");
}

void threshold_select(void) {
    bool should_clear_buffer = false;
    bool got_data = false;

    // The gesture index of the prediction
    int gesture_index;

    // Set up logging.
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        error_reporter->Report(
            "Model provided is schema version DigitalOut led2(LED2);%d not equal "
            "to supported version %d.",
            model->version(), TFLITE_SCHEMA_VERSION);
    }

    // Pull in only the operation implementations we need.
    // This relies on a complete list of all the ops needed by this graph.
    // An easier approach is to just use the AllOpsResolver, but this will
    // incur some penalty in code space for op implementations that are not
    // needed by this graph.
    static tflite::MicroOpResolver<6> micro_op_resolver;
    micro_op_resolver.AddBuiltin(
        tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
        tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                                tflite::ops::micro::Register_MAX_POOL_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                                tflite::ops::micro::Register_CONV_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                                tflite::ops::micro::Register_FULLY_CONNECTED());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                                tflite::ops::micro::Register_SOFTMAX());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                                tflite::ops::micro::Register_RESHAPE(), 1);

    // Build an interpreter to run the model with
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
    tflite::MicroInterpreter* interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors
    interpreter->AllocateTensors();

    // Obtain pointer to the model's input tensor
    TfLiteTensor* model_input = interpreter->input(0);
    if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
        (model_input->dims->data[1] != config.seq_length) ||
        (model_input->dims->data[2] != kChannelNumber) ||
        (model_input->type != kTfLiteFloat32)) {
        error_reporter->Report("Bad input tensor parameters in model");
    }

    int input_length = model_input->bytes / sizeof(float);

    TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
    if (setup_status != kTfLiteOk) {
        error_reporter->Report("Set up failed\n");
    }

    error_reporter->Report("Set up successful...\n");

    while (true) {

        // Attempt to read new data from the accelerometer
        got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

        // If there was no new data,
        // don't try to clear the buffer again and wait until next time
        if (!got_data) {
            should_clear_buffer = false;
        continue;
        }

        // Run inference, and report any error
        TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk) {
        error_reporter->Report("Invoke failed on index: %d\n", begin_index);
        continue;
        }

        // Analyze the results to obtain a prediction
        gesture_index = PredictGesture(interpreter->output(0)->data.f);
        // Clear the buffer next time we read data
        should_clear_buffer = gesture_index < label_num;

        if (gesture_index == 2 /*2 means the gesture of shaking*/) {
            if (Threshold_angle < 60)
                Threshold_angle += 5;
            else if (Threshold_angle == 60)
                Threshold_angle = 30;
        }
    }
}

void publish_Threshold(MQTT::Client<MQTTNetwork, Countdown>* client) {
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "The threshold angle is %d", Threshold_angle);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic1, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void angle_det(Arguments *in, Reply *Out) {
    Thread t2;
    led2 = 1;
    num_tilt = 0;

    t2.start(thread_angle);
    while (1) {
        uLCD.cls();
        uLCD.locate(i, j);
        uLCD.printf("%lf", angle);
        if (num_tilt == 10)
            break;
    }
    ThisThread::sleep_for(100ms);
    t2.terminate();

    // ----------------- wait for command to jump out of this mode --------------------
    char angle_buf[256], angle_outbuf[256]; // for RPC call
    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    memset(angle_outbuf, 0, 256);
    while (1) {         // wait for Python to stop gesture UI mode.
        memset(angle_buf, 0, 256);      // clear buffer
        for(int i=0; i<255; i++) {
            char recv = fgetc(devin);
            if (recv == '\r' || recv == '\n') {
                printf("\r\n");
                break;
            }
            angle_buf[i] = fputc(recv, devout);
        }
        RPC::call(angle_buf, angle_outbuf);
        if (stop == true)
            break;
    }
    stop = false;
    // ---------------------------- end of waiting -------------------------------------

    led2 = 0;
    uLCD.cls();
    Out->putData("Jump out of tilt angle detection mode.");
}

void thread_angle(void) {
    led3 = 0;

    int16_t ref_XYZ[3] = {0};
    int16_t tilt_XYZ[3] = {0};

    // -----------------measure reference accele vector --------------
    printf("measuring the reference acceleration vector\r\n");
    printf("please put mbed on table\r\n");
    led1 = 1;
    ThisThread::sleep_for(2s);      // give user enough time to place mbed on the table
    BSP_ACCELERO_AccGetXYZ(ref_XYZ);
    printf("the reference acceleration vector is (%d, %d, %d)\r\n", ref_XYZ[0], ref_XYZ[1], ref_XYZ[2]);
    led1 = 0;
    // ------------- end of measure reference accele vector ----------

    printf("please start to tilt the board\r\n");
    led3 = 1;
    while (1) {
        BSP_ACCELERO_AccGetXYZ(tilt_XYZ);
        angle = calculate_angle(ref_XYZ, tilt_XYZ);
        if (angle >= Threshold_angle) {
            publish_tilt(pointer_client, angle);
            ++num_tilt;
            printf("tilt angle is %lf degrees for the %d -th time\r\n", angle, num_tilt);
            if (num_tilt == 10)
                break;
        }
        ThisThread::sleep_for(100ms);
    }
    led3 = 0;
}

void publish_tilt(MQTT::Client<MQTTNetwork, Countdown>* client, double angle) {
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "The tilt angle over the threshold one is %lf", angle);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic2, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

double calculate_angle(int16_t ref_XYZ[3], int16_t tilt_XYZ[3]) {
    double angle, cosine_angle;

    // I utilize the inner product of the two vectors
    cosine_angle = (ref_XYZ[0] * tilt_XYZ[0] + ref_XYZ[1] * tilt_XYZ[1] + ref_XYZ[2] * tilt_XYZ[2])
                    / (sqrt(ref_XYZ[0] * ref_XYZ[0] + ref_XYZ[1] * ref_XYZ[1] + ref_XYZ[2] * ref_XYZ[2])
                    * sqrt(tilt_XYZ[0] * tilt_XYZ[0] + tilt_XYZ[1] * tilt_XYZ[1] + tilt_XYZ[2] * tilt_XYZ[2]));
    angle = acos(cosine_angle) / M_PI * 180;

    return angle;
}

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

int PredictGesture(float* output) {
    // How many times the most recent gesture has been matched in a row
    static int continuous_count = 0;
    // The result of the last prediction
    static int last_predict = -1;

    // Find whichever output has a probability > 0.8 (they sum to 1)
    int this_predict = -1;
    for (int i = 0; i < label_num; i++) {
        if (output[i] > 0.8) this_predict = i;
    }

    // No gesture was detected above the threshold
    if (this_predict == -1) {
        continuous_count = 0;
        last_predict = label_num;
        return label_num;
    }

    if (last_predict == this_predict) {
        continuous_count += 1;
    } else {
        continuous_count = 0;
    }
    last_predict = this_predict;

    // If we haven't yet had enough consecutive matches for this gesture,
    // report a negative result
    if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
        return label_num;
    }
    // Otherwise, we've seen a positive result, so clear all our variables
    // and report it
    continuous_count = 0;
    last_predict = -1;

    return this_predict;
}
void stop_mode(Arguments *in, Reply *Out) {
    stop = true;
}