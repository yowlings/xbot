#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "nlp/nlp_feedback.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "please enter the your question\n";
    return -1;
  }
  TuLingRobot tuling;
  tuling.setAskJson(argv[1]);
  tuling.callTulingApi();
  tuling.textFromJson();
  return 0;
}
