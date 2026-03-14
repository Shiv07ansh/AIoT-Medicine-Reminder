#ifndef CLASS_LABELS_H
#define CLASS_LABELS_H

const char* class_labels[] = {
  "ask_med_details", // 0
  "ask_schedule",    // 1
  "ask_time",        // 2
  "confirm_taken",   // 3
  "deny_taken",      // 4
  "irrelevant",      // 5
  "notify_sos",      // 6
  "remind_later"     // 7
};

const int NUM_CLASSES = sizeof(class_labels) / sizeof(class_labels[0]);

#endif // CLASS_LABELS_H