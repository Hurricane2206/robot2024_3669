#include <rev/CANSparkMax.h>

class Intake{
public:
    Intake(){
        loaded = 0;
    }
    void load(){}
    void unload(){}
    void aim(){}
    void shoot(){}
private:
    bool loaded;
};