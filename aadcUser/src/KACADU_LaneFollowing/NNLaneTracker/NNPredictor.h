#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern "C" { 
    #include <lua.h> 
    #include <lauxlib.h> 
    #include <lualib.h> 
    
    #include "TH/TH.h"
    #include "luaT.h"
    #include "lauxlib.h"
    #include "TH/generic/THTensor.h"
 
} 


#define LUA_FILE "/home/aadc/Documents/nn.lua"
#define IMAGE_WIDTH 640.0
#define IMAGE_HEIGHT 480.0
#define LINE_POS 0.2
#define LABELS 16.0

class NNPredictor {
public:

	NNPredictor();
	~NNPredictor();

    	void getLanePoint(cv::Mat image, int &x, int &y);

	cv::Mat getResultImage();
private:
	
	lua_State* mLua;
	
	cv::Mat mImage;


	lua_State * load_lua_file (const char *filename);
	THDoubleTensor * opencv_img_to_tensor(cv::Mat img);
	
};

