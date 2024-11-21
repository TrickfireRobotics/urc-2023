# Running on Rover

Instructions on running code on Viator's computer

1. Setup router and connect to the wifi network `trickfirerouter` on your computer (the router's password is on the router itself).
  - To get wifi to access Github, connect the ethernet cable to the router and the ethernet plug that is located overhead above the tables in the middle.
2. Turn the rover computer on by plugging in the C-port charger. You will know the computer is on if the LED lights up.
3. Run the commands below in Powershell if on windows or in your OS's terminal. When prompted for a password, please ask a software lead for it.

```
> ssh trickfire@192.168.0.145
In Bash Now
> cd ~/urc-2023
> sudo ./container_launch.sh
In Docker Now
> cd /home/trickfire/urc-2023
> ./build.sh
> ./launch.sh
```