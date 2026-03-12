#include "ForceShoePlugin.h"

int main(int argc, char * argv[])
{

  mc_force_shoe_plugin::ForceShoePluginSchema c;
  c.load("../../tests/etc/ForceShoePlugin.yaml");
  return 0;
}
