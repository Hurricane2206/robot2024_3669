#include <rev/CANSparkMax.h>

class Arm{
public:
    Arm(){
        shootMode = 0;
    }
    void load(){}
    void unload(){}
    void setDistance(float inches){}
    void toggleShoot(){
        if (shootMode){
            shootMode = 0;
        }
        else {
            shootMode = 1;
        }
    }
private:
    bool shootMode;
};