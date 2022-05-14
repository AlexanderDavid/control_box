# Control Box

This repo provides the code and installation instructions for creating a
Raspberry Pi based simple controller for ROS projects. My use case for this is
to provide easy switching between motion planners and an emergency stop for a
robot interaction study I am conducting. Although, this could be used in a
myriad of other projects. This box simply reads some input from the GPIO (either
a button or a rotary switch) and then publishes this onto ROS. 

Additionally, I have included the specific products that I have used to create 
my version of this box. However, you should be fine to use whatever so long as
you can plumb it up correctly

## Software
- [Install ROS on RaspiOS Buster](https://varhowto.com/install-ros-noetic-raspberry-pi-4/#ROS_Noetic_Raspberry_Pi)
- [Automatically starting the node](http://docs.ros.org/en/jade/api/robot_upstart/html/)


## Hardware
### Buy List
- [Raspberry Pi](https://www.amazon.com/gp/product/B089ZSGF8M/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [Perf Board](https://www.amazon.com/gp/product/B07FFDFLZ3/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&th=1)
- [Project Box](https://www.amazon.com/dp/B09B6RB6MR?ref=ppx_yo2ov_dt_b_product_details&th=1)
- [Panel Mounted USB-C](https://www.amazon.com/dp/B08HS6X44P?ref=ppx_yo2ov_dt_b_product_details&th=1)
- [Panel Mounted RJ45](https://www.amazon.com/dp/B08H8JM451?ref=ppx_yo2ov_dt_b_product_details&th=1)
- [Rocker Switches](https://www.amazon.com/dp/B07MFQ4TNK?ref=ppx_yo2ov_dt_b_product_details&th=1)
- [Emergency Stop](https://www.amazon.com/dp/B07WTL3KPB?ref=ppx_yo2ov_dt_b_product_details&th=1)
- [Rotary Switch](https://www.amazon.com/dp/B07JKQ7YPP?psc=1&ref=ppx_yo2ov_dt_b_product_details)