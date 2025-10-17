#include <iostream>
#include <signal.h>

#include <leap_hand_utils/leap_controller.h>

/*****************************************************/
// This can control and query the LEAP Hand

// I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

//Allegro hand conventions:
//0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
//http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

//LEAP hand conventions:
//180 is flat out for the index, middle, ring, finger MCPs.
//Applying a positive angle closes the other joints more and more.
/*****************************************************/

volatile sig_atomic_t flag = 0;
void keyboard_interrupt(int sig) {
    flag = 1;
}

int main()
{
    // Register signals 
    signal(SIGINT, keyboard_interrupt);

    LeapController leap_hand {};
    while (1) {
        if (flag) {
            printf("\nShutting down\n");
            break;
        }

        leap_hand.set_allegro(Eigen::MatrixXd::Zero(1, 16));
        std::cout << "Position: " << leap_hand.read_pos() << '\n';
    }
}