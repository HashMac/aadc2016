#include <iostream>

#include "host.h"


#include <dlfcn.h>

using namespace std;
using namespace cv;

/*
extern "C" { 
    #include <lua.h> 
    #include <lauxlib.h> 
    #include <lualib.h> 
    
    #include "TH/TH.h"
    #include "luaT.h"
    #include "lauxlib.h"
    #include "TH/generic/THTensor.h"
 
} */

//loads and inits the lua file
lua_State * load_lua_file (const char *filename){
    
  cout << "** Test Lua embedding" << endl;
  cout << "** Init Lua" << endl;

    void *hdl = dlopen("liblua5.1.so", RTLD_NOW | RTLD_GLOBAL);
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
    cerr << "Something went wrong loading the chunk (syntax error?)" << endl;
    cerr << lua_tostring(L, -1) << endl;
    lua_pop(L,1);
  }

  cout << "** Execute the Lua chunk" << endl;
  if (lua_pcall(L,0, LUA_MULTRET, 0)) {
    cerr << "Something went wrong during execution" << endl;
    cerr << lua_tostring(L, -1) << endl;
    lua_pop(L,1);
  }
    
    return L;
}


THDoubleTensor * opencv_img_to_tensor(Mat img){
    
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


int main2()
{
    
    lua_State *L = load_lua_file("/home/aadc/Documents/hello.lua");

    //Load pretrained NN
    lua_getglobal(L, "load_net");
    lua_pushnumber(L, 5);
    lua_pcall(L, 1, 1, 0);
    lua_pop(L,1);


   
    //Convert opencv image to Torch Tensor
    //THDoubleTensor *testTensor = THDoubleTensor_newWithSize3d(3,480,640);
    Mat img;
    img = imread("/home/aadc/Documents/testimgs/19332064.png", CV_LOAD_IMAGE_COLOR);
    THDoubleTensor *testTensor = opencv_img_to_tensor(img);


    //Get prediction for image
    lua_getglobal(L, "myluafunction");
    luaT_pushudata(L,testTensor, "torch.DoubleTensor");
    lua_pcall(L, 1, 1, 0);
    int prediction = lua_tonumber(L, -1);
    lua_pop(L,1);

    
    cout << "The return value of the function was " << prediction << endl;
    lua_pop(L,1);

        
    cout << "** Release the Lua enviroment" << endl;
    lua_close(L);
    
}

int main(){


	return 0;
}
