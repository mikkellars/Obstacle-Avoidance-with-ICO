#g++ main.cpp -o main.out \
#-Wall \
#-pthread \
#-lrt \
#-lm \
#-lpigpio \

g++ main.cpp -o main.out \
-Wall \
-lmatrix_creator_hal

echo "To run, type:"
echo "./main.out"
