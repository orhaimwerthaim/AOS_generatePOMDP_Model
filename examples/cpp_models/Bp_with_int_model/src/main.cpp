#include <despot/simple_tui.h>
#include "Bp_with_int_model.h"

using namespace despot;

class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = new Bp_with_int_model();
    return model;
   }

  void InitializeDefaultParameters() {
     Globals::config.num_scenarios = 100;
  }
};

int main(int argc, char* argv[]) {
  return TUI().run(argc, argv);
}
