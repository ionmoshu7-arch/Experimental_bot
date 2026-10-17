// Interface RemoteXY du telephone, generee par l'editeur remotexy.com
// (version gratuite). Ne pas modifier a la main : refaire l'ecran dans
// l'editeur et recoller ici le bloc entre les deux #pragma.
//   joystick (revient au centre), curseur de vitesse 0..100,
//   selecteur 3 positions A = manuel, B = suivi de ligne, C = navigation,
//   bouton STOP rouge.

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = // 58 bytes V19
{ 255,5,0,0,0,51,0,19,0,0,0,0,31,1,106,200,1,1,4,0,
5,2,13,60,60,32,2,26,31,4,75,22,17,50,0,2,26,1,63,116,
24,24,0,1,31,83,84,79,80,0,3,17,106,16,44,3,2,26 };

struct {
  // input variables
  int8_t joystick_x;      // from -100 to 100
  int8_t joystick_y;      // from -100 to 100
  int8_t slider_vitesse;  // from 0 to 100
  uint8_t button_stop;    // =1 if button pressed, else =0
  uint8_t selecteur_mode; // 0 = manuel, 1 = ligne, 2 = nav
  // other variable
  uint8_t connect_flag;   // =1 if wire connected, else =0
} RemoteXY;
#pragma pack(pop)
