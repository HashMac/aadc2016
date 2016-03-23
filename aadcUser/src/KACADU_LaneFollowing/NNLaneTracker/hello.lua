require "nn";
require "image";

print("Hello from Lua")
--net = nn.Sequential();


function init_net(test)
    
    net = nn.Sequential()
    net:add(nn.SpatialConvolution(3,6,5,5))
    net:add(nn.SpatialMaxPooling(2,2,2,2))
    net:add(nn.SpatialConvolution(6,16,5,5))
    net:add(nn.SpatialMaxPooling(2,2,2,2))
    net:add(nn.SpatialConvolution(16,128,5,5))
    net:add(nn.SpatialMaxPooling(2,2,1,1))
    -- 37 == ((160 -4 ) % 2 -4 ) % 2 (-4 == Convolution without padding stride 1, %2 == MaxPooling with stride 2 (2x2) )
    net:add(nn.View(128*32*22))
    net:add(nn.Linear(128*32*22,1024))
    net:add(nn.Linear(1024,84))
    net:add(nn.Linear(84,20))
    net:add(nn.LogSoftMax())  
    
    return 0 
    
end 


function load_net(test)
    
    filename = "/home/aadc/Documents/second_model.net"
    net = torch.load(filename)
    
    
end    

function myluafunction(tensor)
  
    
    img_small = image.scale(tensor,160,120)
    
    
    timer = torch.Timer()
    prediction = net:forward(img_small)
    confidences, indices = torch.sort(prediction, true)
    print('Time elapsed : ' .. timer:time().real .. ' seconds')
    print("prediction:", indices[1])
        
    return indices[1] 
    
end

