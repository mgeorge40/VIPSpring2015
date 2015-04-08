function analyzeColor(imageName)
getPoints(imageName)
disp('Click 5 points on the color sample you want to analyze (suggested: four corners and the middle).')
end

function points = getPoints(imageName)

clc
close all
figure('WindowButtonDownFcn',@wbdcb)
RGB = imread(imageName);
image(RGB);
axis image;
axis off;
h = imgca;
points = [];
click = 0;
    function wbdcb(src,callbackdata)
    if strcmp(get(src,'SelectionType'),'normal')
        cp = get(h,'CurrentPoint');
        x = cp(1,1); %these give you the x and y coordinates that it draws the point at
        y = cp(1,2);
        x = floor(x);
        y = floor(y);
        click = click+1
        points(1, click) = x;
        points(2, click) = y;
        if click == 5
            makeRed(imageName, points)
        end
    end
end
end

function makeRed(imageName, points)
Rvalues = 0;
Gvalues = 0;
Bvalues = 0;
RGB = imread(imageName);
RGB = double(RGB);
[l, w, c] = size(RGB);
red = RGB(:,:,1);
green = RGB(:,:,2);
blue = RGB(:,:,3);
for num = 1:5
    Rvalues = Rvalues + RGB(points(2, num), points(1, num), 1);
    Gvalues = Gvalues + RGB(points(2, num), points(1, num), 2);
    Bvalues = Bvalues + RGB(points(2, num), points(1, num), 3);
end
Raverage = round(Rvalues/5);
Gaverage = round(Gvalues/5);
Baverage = round(Bvalues/5);
tol = input('Tolerance?\n');
maskR = red <= (Raverage + tol) & red >= (Raverage - tol);
maskG = green <= (Gaverage + tol) & green >= (Gaverage - tol);
maskB = blue <= (Baverage + tol) & blue >= (Baverage - tol);
red(maskR & maskG & maskB) = 255;
green(maskR & maskG & maskB) = 0;
blue(maskR & maskG & maskB) = 0;
out = uint8(cat(3, red, green, blue));
imshow(out)

end