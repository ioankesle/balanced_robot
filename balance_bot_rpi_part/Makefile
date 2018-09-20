
all: demo_dmp bot

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h pi2c.h
CMN_OBJS = I2Cdev.o MPU6050.o pi2c.o
DMP_OBJS = demo_dmp.o 
BT_OBJS = bot.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

CXXFLAGS = -DDMP_FIFO_RATE=1 -Wall -fpermissive -lpthread -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`

$(CMN_OBJS) $(DMP_OBJS) $(PI2C_OBJS) $(BT_OBJS): $(HDRS)

demo_dmp: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ bot.cpp $^ -lm -lbluetooth -lrt -pthread

bot: $(CMN_OBJS) $(DMP_OBJS) $(BT_OBJS)
	$(CXX) -o $@ $^ -lm -lbluetooth -lrt -pthread

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(BT_OBJS) $(PI2C_OBJS) bot demo_dmp