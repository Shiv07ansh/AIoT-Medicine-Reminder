/*
 * labels.h
 * =====================================================================
 * SLU intent class labels — must match training order exactly.
 * =====================================================================
 */

#pragma once

#define NUM_CLASSES 8

static const char* class_labels[NUM_CLASSES] = {
  "ask_med_details",  // 0
  "ask_schedule",     // 1
  "ask_time",         // 2
  "confirm_taken",    // 3
  "deny_taken",       // 4
  "irrelevant",       // 5
  "notify_sos",       // 6
  "remind_later"      // 7
};
