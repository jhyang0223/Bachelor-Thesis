# SUB-1GHz Wireless Internet Extender for Providing Wireless LAN Services in Outdoor Working Environments

Jihyeok Yang, Jimin Cheon, Youngtak Kim

## abstract
In outdoor work environments where wired LAN access is not possible within a 100m radius, such as rural rice paddies, fields, and construction sites, access to the Internet should be made using expensive data from Eggs or smartphone LTE. Alternatively, a WiFi extender in the 2.4 GHz band should be used, but due to the feature of the frequency, the operating distance is short due to low diffraction. This paper proposes an Internet relay function using an RF module in the 900 MHz band with a radius of 1 km. **We implemented a TI CC1310 Device Driver based on 1:1 TDD Communication and a Linux Network Device Driver based on SPI Communication with CC1310 that includes Packet's segmentation & assembly function and Flow Control function.** This makes it easier to work in a better environment by establishing an Internet use environment for those working in rural rice paddies, fields, and construction sites with a shorter operating distance than WiFi extenders that complements indoor shaded areas.

## Contribution
- Jihyeok Yang: 
  * Implemented a TI CC1310 Device Driver based on 1:1 TDD Communication 
  * Implemented a Linux Network Device Driver based on SPI Communication with CC1310 that includes Packet's segmentation & assembly function and Flow Control function.
  
- Jimin Cheon: 
  * Configured Rasberry Pi AP mode

## System Structure
### System Diagram
![system diagram](https://user-images.githubusercontent.com/44808660/189515043-303396d8-236f-48d3-b7bc-66a6c4b3d711.png)

### System Functional Block Diagram
![System Functional Block Diagram](https://user-images.githubusercontent.com/44808660/189515052-2035c884-06cc-4057-8b93-37118d70e423.png)

We implemented orange blocks in System Functional Block Diagram except for the Radius server.  
In the operation of our system, it worked without a Radius server. But the professor said it should be implemented later. 
## Experiment and Result
Theoretically, our CC1310 settings enabled communication at 500 kbps. However, as a result of running the benchmark app with a connected smartphone, it only came out at a speed of 40 to 70 kbps. This is because the network control packet is not appropriately controlled and is transmitted and received through RF communication.
