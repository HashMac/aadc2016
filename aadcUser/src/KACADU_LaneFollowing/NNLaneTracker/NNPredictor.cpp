#include "NNPredictor.h"

#include <iostream>
#include <dlfcn.h>


using namespace std;
using namespace cv;


NNPredictor::NNPredictor(){

    mLua = load_lua_file(LUA_FILE);

    //Load pretrained NN
    lua_getglobal(mLua, "load_net");
    lua_pushnumber(mLua, 5);
    lua_pcall(mLua, 1, 1, 0);
    lua_pop(mLua,1);


}


NNPredictor::~NNPredictor(){

    cout << "** Release the Lua enviroment" << endl;
    //lua_close(mLua);

}


void NNPredictor::getLanePoint(cv::Mat img, int &x, int &y){

    mImage = img;

    THDoubleTensor *testTensor = opencv_img_to_tensor(img);

    //Get prediction for image
    lua_getglobal(mLua, "forward");
    luaT_pushudata(mLua,testTensor, "torch.DoubleTensor");
    lua_pcall(mLua, 1, 1, 0);
    int prediction = lua_tonumber(mLua, -1);
    lua_pop(mLua,1);

    cout << "The return value of the function was " << prediction << endl;
    //lua_pop(mLua,1);


    y =  (int) (IMAGE_HEIGHT  * (1.0 - LINE_POS) ) ;
    x =  (int) (IMAGE_WIDTH - (IMAGE_WIDTH / LABELS) * prediction + ((IMAGE_WIDTH / LABELS) * 0.5) ) ;

	
    circle( mImage, cv::Point2f(x,y), 5, CV_RGB( 128, 128, 255 ), -1 );
}

cv::Mat NNPredictor::getResultImage()
{
	return mImage;
}


lua_State * NNPredictor::load_lua_file (const char *filename){

	cout << "** Test Lua embedding" << endl;
	cout << "** Init Lua" << endl;

	void *hdl = dlopen("libluajit.so", RTLD_NOW | RTLD_GLOBAL);
	if(hdl == 0) {
		cout << dlerror() << endl;
	} else {
		cout << "loaded lua library" << endl;
	}


	lua_State *L;
	L = luaL_newstate();
	cout << "** Load the (optional) standard libraries, to have the print function" << endl;
	luaL_openlibs(L);
	cout << "** Load chunk. without executing it" << endl;
	if (luaL_loadfile(L, filename)) {
		cout << "Something went wrong loading the chunk (syntax error?)" << endl;
		cout << lua_tostring(L, -1) << endl;
		lua_pop(L,1);
	}

	cout << "** Execute the Lua chunk" << endl;
	if (lua_pcall(L,0, LUA_MULTRET, 0)) {
		cout << "Something went wrong during execution" << endl;
		cout << lua_tostring(L, -1) << endl;
		lua_pop(L,1);
	}

    	return L;


}

THDoubleTensor * NNPredictor::opencv_img_to_tensor(cv::Mat img){

  
    THDoubleTensor *testTensor = THDoubleTensor_newWithSize3d(3,480,640); //Initialize 3D tensor with size * 3 (R,G,B).
    double *testTensorData = THDoubleTensor_data(testTensor);
    
    double r,g,b;
    
    uint8_t* pixelPtr = (uint8_t*)img.data;
    int cn = img.channels();
    Scalar_<uint8_t> bgrPixel;

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            b = pixelPtr[i*img.cols*cn + j*cn + 0]; // B
            g = pixelPtr[i*img.cols*cn + j*cn + 1]; // G
            r = pixelPtr[i*img.cols*cn + j*cn + 2]; // R

            
             testTensorData[0* (img.rows*img.cols) + img.cols * i + j ] = r/255.0 ;
             testTensorData[1* (img.rows*img.cols) + img.cols * i + j ] = g/255.0 ;
             testTensorData[2* (img.rows*img.cols) + img.cols * i + j ] = b/255.0 ;
             
        }
    }
    
    return testTensor;

}












