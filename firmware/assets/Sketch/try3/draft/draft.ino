/* 
  Sketch: TFLite Micro inference on ESP32-S3 with embedded model_data.h

  Sections:
  1. Includes & model header
  2. Global declarations
  3. Helper functions (print dims, print memory)
  4. setup(): load model from flash, allocate arena, build interpreter
  5. loop(): fill input, run inference, print result
  6. Notes on adjusting resolver ops and input data
*/

// 1. Include necessary libraries
#include <Arduino.h>
#include "SPIFFS.h"                              // For mounting SPIFFS
#include <tflm_esp32.h>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
//#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "model_data_710pm.h"
#include "vocab_710pm.h"
#include <string>
#include "labels.h"
const tflite::Model* model = tflite::GetModel(model_data_710pm);
// 2. Global declarations


// Globals: only pointers, no large arrays
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input_tensor = nullptr;
TfLiteTensor* output_tensor = nullptr;

uint8_t* tensor_arena = nullptr;
constexpr int kTensorArenaSize = 32 * 1024;  // start very small; adjust after tests

const int kMaxLen = 15;
int input_data[15]={0};  // buffer to hold tokenized + padded input
//int8_t input_data[15] = {0};



// ==================== FUNCTIONS ===========================

void tokenize_and_pad(const String& sentence, int* input_data, int max_len) {
  String word = "";
  int pos = 0;

  for (int i = 0; i <= sentence.length(); i++) {
    char c = sentence.charAt(i);

    // Treat space or end of sentence as word boundary
    if (c == ' ' || i == sentence.length()) {
      if (word.length() > 0) {
        word.toLowerCase();
        int token = word_index.count(word) ? word_index.at(word) : word_index.at("<OOV>");

        //int token = word_index.count(word) ? word_index[word] : word_index["<OOV>"];
        if (pos < max_len) {
          input_data[pos++] = (int)token;
        }
        word = "";
      }
    } else {
      word += c;
    }
  }

  // Pad the rest with zeros
  while (pos < max_len) {
    input_data[pos++] = 0;
  }
}


//===================== FUNCTIONS =======================
//=============== VOID SETUP =================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== Setup start ===");

  // 1. Print free heap
  Serial.print("Free heap before arena malloc: ");
  Serial.println(ESP.getFreeHeap());

  // 2. Allocate tensor arena in heap
  tensor_arena = (uint8_t*) malloc(kTensorArenaSize);
  if (!tensor_arena) {
    Serial.print("ERROR: malloc tensor_arena failed. Free heap: ");
    Serial.println(ESP.getFreeHeap());
    while (1);
  }
  Serial.print("tensor_arena malloc OK, size=");
  Serial.print(kTensorArenaSize);
  Serial.print(", Free heap after malloc: ");
  Serial.println(ESP.getFreeHeap());

  // 3. Load model from flash

  if (!model || model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERROR: Model invalid or schema mismatch");
    while (1);
  }
  Serial.println("Model loaded from flash");

  // 4. Build resolver (add only ops needed)
  static tflite::MicroMutableOpResolver<20> resolver;  // adjust count
  resolver.AddCast();
  resolver.AddConcatenation();
  resolver.AddDequantize();
  resolver.AddFill();
  resolver.AddFullyConnected();
  resolver.AddGather();
  resolver.AddPack();
  resolver.AddQuantize();
  resolver.AddShape();
  resolver.AddSoftmax();
  resolver.AddStridedSlice();
  resolver.AddTranspose();
  resolver.AddMean();
  // Add others as needed...

  // 5. Create interpreter
  static tflite::MicroInterpreter static_interpreter(
    model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;
  if (!interpreter) {
    Serial.println("ERROR: Interpreter creation failed");
    while (1);
  }

  // 6. Allocate tensors
  TfLiteStatus status = interpreter->AllocateTensors();
  if (status != kTfLiteOk) {
    Serial.print("ERROR: AllocateTensors failed with arena size ");
    Serial.println(kTensorArenaSize);
    while (1);
  }
  Serial.println("Tensors allocated");



  // 7. Obtain tensors
  input_tensor = interpreter->input(0);
  output_tensor = interpreter->output(0);
  if (!input_tensor || !output_tensor) {
    Serial.println("ERROR: Null tensor pointers");
    while (1);
    }

 
  Serial.print("Input tensor type: ");
  Serial.println(input_tensor->type);
  Serial.println("=== Setup complete ===");
  Serial.println("Ready for input. Type a sentence:");
}

//======================== VOID LOOP ==================
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    Serial.print("Received: ");
    Serial.println(input);


    // Tokenize & pad
    tokenize_and_pad(input, input_data, 15);

    // Print Tokens
     Serial.print("Input tokens: ");
     for (int i = 0; i < 15; i++) {
        Serial.print(input_data[i]);
        Serial.print(" ");
      }
     Serial.println();


    // Copy input into tensor
    for (int i = 0; i < 15; i++) {
      interpreter->input(0)->data.i32[i] = input_data[i];
      //interpreter->input(0)->data.int32[i] = input_data[i];
    }

    // Run inference
    if (interpreter->Invoke() != kTfLiteOk) {
      Serial.println("ERROR: Invoke failed");
      return;
    }

    TfLiteTensor* output = interpreter->output(0);

    int max_index = 0;
    float max_score = output->data.f[0];

    // Loop through logits to find the max
    for (int i = 1; i < NUM_CLASSES; i++) {
      float score = output->data.f[i];
      Serial.printf("Output[%d] = %.5f\n", i, score);
      if (score > max_score) {
        max_score = score;
        max_index = i;
    }
 }

    // Output predicted class
     Serial.printf("Predicted class: %s (index %d, score %.4f)\n",
              class_labels[max_index], max_index, max_score);
    switch (max_index) {
  case 0: // ask_med_details
    triggerAskMedDetails();
    break;
  case 1: // ask_schedule
    triggerAskSchedule();
    break;
  case 2: // ask_time
    triggerAskTime();
    break;
  case 3: // confirm_taken
    triggerConfirmTaken();
    break;
  case 4: // deny_taken
    triggerDenyTaken();
    break;
  case 5: // irrelevant
    // maybe do nothing or a generic response
    triggerIrrelevant();
    break;
  case 6: // notify_sos
    triggerNotifySOS();
    break;
  case 7: // remind_later
    triggerRemindLater();
    break;
  default:
    // unknown
    break;
}

    /*
    // Find predicted class
    int outputLength = output_tensor->dims->data[1];
    int8_t* out_buf = output_tensor->data.int8;
    int pred = 0;
    int8_t max_val = out_buf[0];
    for (int i = 1; i < outputLength; i++) {
      if (out_buf[i] > max_val) {
        max_val = out_buf[i];
        pred = i;
      }
    }

    Serial.print("Predicted class: ");
    Serial.println(pred);
    */
  }
}
