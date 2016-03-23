#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

extern "C" { 
    #include <lua.h> 
    #include <lauxlib.h> 
    #include <lualib.h> 
    
    #include "TH/TH.h"
    #include "luaT.h"
    #include "lauxlib.h"
    #include "TH/generic/THTensor.h"
 
} 



using namespace cv;


lua_State * load_lua_file (const char *filename);
THDoubleTensor * opencv_img_to_tensor(Mat img);
int main2();
