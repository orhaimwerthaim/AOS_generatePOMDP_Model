#include <despot/simple_tui.h>
#include "turtleBotVisitLocationsPO_Noisy.h"

using namespace despot;

class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = new TurtleBotVisitLocationsPO_Noisy();
    return model;
   }

  void InitializeDefaultParameters() {
     Globals::config.num_scenarios = 100;
  }
};

int main(int argc, char* argv[]) {
  return TUI().run(argc, argv);
}
