#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <fcntl.h>
#include <tr1/memory>
#include <libusb-1.0/libusb.h>

// Absolute path to the source folder of the package
std::string path = "/home/summit/catkin_ws/src/new_hardware_detection/src/";

// Structure for storing tools and new devices' IDs from this tools
struct Tool_Difference {
	std::string name;
	std::string newID[10];
	int id_count;
};
bool short_output = false;

void chatterCallback(const std_msgs::String::ConstPtr&);
bool isGood(std::string, std::string, std::string = "");
bool inWhiteList(std::string, std::string);
std::string exec(const char*);
const char* decision_to_string(bool);
const char* class_to_string(uint8_t);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main function. Calls callback function in the loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(int argc, char **argv)
{

	ros::init(argc, argv, "analyse_hardware");
	ros::NodeHandle n;


	if (argc > 1) {
	  for (int i = 1; i < argc; i++) {
		ROS_DEBUG("Argument %d: %s", i, argv[i]);
		if (strcmp(argv[i], "--short") == 0)
		  short_output = true;
		else {
		  ROS_WARN("Possible options: --short");
		  break;
		  }
	  }
	}

	ros::Subscriber sub = n.subscribe("new_devices", 100, chatterCallback);

	ros::spin();

	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Callback function to handle message from the detection node
// Structure or the message:
//
// tool_name1
// new_device1
// new_device2
// ...
// tool_name2
// new_device1
// ...
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	system("reset"); // clear the terminal, delete all previous infomation
		
	std::string cmd; // command for linux terminal

	ROS_DEBUG("Message from detection node: \n %s", msg->data.c_str());

	//Here starts dividing message to the pairs tool-devices
	std::string tools = "lsusb lspci"; // we need to know what tools are we using
	Tool_Difference tool_diff[2];

	int first = 0, tools_count = 0, id_count = 0;
	std::string line;
	for (int i = 0; i < msg->data.length(); i++) {
	  if (msg->data[i] == '\n') {
		line = msg->data.substr(first, i - first);
		if (tools.find (line) != std::string::npos) {
		  tool_diff[tools_count].name = line;
		  tools_count++;
		  tool_diff[tools_count].id_count = id_count;
		  id_count = 0;
		  }
		else {
		  tool_diff[tools_count - 1].newID[id_count] = line;
		  id_count++;
		  }
		first = i + 1;
		}		
	  }

	// Here starts the analysis of new devices

    int count = 1; // Counter of new devices
    uint16_t vid, pid; 
    bool decision = true; // true - device is good, false - device is bad
  	
    for (int i=0; i < tools_count; i++) {

	  // Analysis of new usb devices
      if (tool_diff[i].name == "lsusb") {
	  
		// Using library libusb-1.0
		libusb_device **devs; // here we will store all connected usb devices
		libusb_device_handle *dev_handle;
		libusb_context *ctx = NULL; // a libusb session
		libusb_device_descriptor desc;
		libusb_config_descriptor *config;
		const libusb_interface *inter; 
		const libusb_interface_descriptor *inter_desc;
  
		int r; // for return values
		ssize_t cnt; 
		r = libusb_init(&ctx); // initialize linusb session
		if (r < 0) {
		  	ROS_ERROR("Error initialising libusb");
	  		break;
	  		}
        libusb_set_debug(ctx, 3); //set verbosity level to 3, as suggested in the documentation
        cnt = libusb_get_device_list(ctx, &devs);
		if (cnt < 0) {
		  	ROS_ERROR("Failed to get device list");
	  		break;
	  		}

	  // Analyse new devices one by one
	  for (int j = 0; tool_diff[i].newID[j] != ""; j++) { 

		ROS_INFO("New USB device %d", count);
			
		vid = std::strtol (tool_diff[i].newID[j].substr(0, 4).c_str(), 0, 16);
		pid = std::strtol (tool_diff[i].newID[j].substr(5, 4).c_str(), 0, 16);
		ROS_DEBUG("VID: %04x, PID: %04x", vid, pid);

		int was_before = 0; // we must consider that there can be devices with the same ID
		for (int k = 0; k < j; k++) {
		  if (tool_diff[i].newID[j] == tool_diff[i].newID[k])
			was_before++;
		  }
		ROS_DEBUG("There were %d such devices before", was_before);

		// Find devices with the same VID and PID
		for (int k = 0; k < cnt; k++) {
		  r = libusb_get_device_descriptor(devs[k], &desc);
		  if (r < 0) {
		  	ROS_ERROR("Failed to get device descriptor");
	  		break;
	  		}
		  if (desc.idVendor == vid & desc.idProduct == pid) {
			if (was_before > 0) {
			  was_before--;
			  continue;
			  }
			libusb_get_config_descriptor(devs[k], 0, &config); 
			r = libusb_open (devs[k], &dev_handle); // open device, get it's handler
	 		if (r < 0) {
			  ROS_ERROR("Failed to open device");
			  break;
			  }
			ROS_INFO_COND(short_output == false, "Bus: \t\t%d", libusb_get_bus_number(devs[k]) );
			ROS_INFO_COND(short_output == false, "Address: \t%d", libusb_get_device_address(devs[k]) );
			break;
			}
		  }
	  if (r < 0) { // if something was wrong, go to the next device
	    continue;
	    }

	  // Get all information we want from the device
	  unsigned char manufacturer[42] = "";
	  if (desc.iManufacturer) {
	    r = libusb_get_string_descriptor_ascii(dev_handle, desc.iManufacturer, manufacturer, 42);
	    if (r < 0) {
	      ROS_ERROR("Failed to get manufacturer name");
	      }
	    }
      ROS_INFO_COND(short_output == false, "Vendor: \t\t%04x %s", vid, manufacturer);

	  unsigned char product[42] = "";
	    if (desc.iProduct) {
	    r = libusb_get_string_descriptor_ascii(dev_handle, desc.iProduct, product, 42);
	    if (r < 0) {
		  ROS_ERROR("Failed to get product name");
		  }
		}
      ROS_INFO("Product: \t\t%04x %s", pid, product);

	  unsigned char serial[42] = "";
	  if (desc.iSerialNumber) {
		r = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, serial, 42);
		if (r < 0) {
		  ROS_ERROR("Failed to get serial number");
		  continue;
		  }
		ROS_INFO_COND(short_output == false, "Serial Number: \t\t%s", serial);
		}
			
      ROS_INFO_COND(short_output == false, "Device Class: \t\t%d %s", desc.bDeviceClass, class_to_string(desc.bDeviceClass));
	  if (desc.bDeviceClass != 0) { // if class in defined on the device descriptor level
		ROS_INFO_COND(short_output == false, "Device Subclass: \t%d", desc.bDeviceSubClass);
		ROS_INFO_COND(short_output == false, "Device Protocol: \t%d", desc.bDeviceProtocol);
		decision = isGood(std::to_string(desc.idVendor), class_to_string(desc.bDeviceClass), reinterpret_cast <const char*> (serial));
		}

	  else { // if class in defined on the interface descriptor level
		// We want to know information about every interface, but normally there is only one
		ROS_INFO_COND(short_output == false, "Number of interfaces: \t%d", config->bNumInterfaces);
		for (int i=0; i < (int) config->bNumInterfaces; i++) {
	      inter = &config->interface[i];
	      ROS_INFO_COND(short_output == false, "Interface %d", i + 1);
	      ROS_INFO_COND(short_output == false, "  Number of alternative settings: %d", inter->num_altsetting);
	      for (int j = 0; j < (int) inter->num_altsetting; j++) {
			inter_desc = &inter->altsetting[j];
			ROS_INFO_COND(short_output == false, "  Alternative setting %d", j + 1);
			ROS_INFO_COND(short_output == false, "    Interface Class: \t\t%d %s", inter_desc->bInterfaceClass, class_to_string(inter_desc->bInterfaceClass));
			ROS_INFO_COND(short_output == false, "    Interface Subclass: \t%d", inter_desc->bInterfaceSubClass);
			ROS_INFO_COND(short_output == false, "    Interface Protocol: \t%d", inter_desc->bInterfaceProtocol);
			if ( isGood(std::to_string(desc.idVendor), class_to_string(inter_desc->bInterfaceClass), reinterpret_cast <const char*> (serial)) == false)
			  decision = false;
			}
		  }
		}

	  ROS_INFO("DECISION: %s", decision_to_string(decision));
	  std::cout<<"\n";

	  if (decision == false) { // if device is bad
		//TODO: block device, message to administrator
		//r = libusb_set_configuration(dev_handle, -1);
		//if (r == 0)
		//  ROS_INFO("Device is put into unconfigured state");
		}

	  libusb_free_config_descriptor(config);	
	  libusb_close(dev_handle); // close device
	  count++;
	  decision = true;	  
      }	

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx); // close libusb session
    }


	// Analysis of pci devices
	else if (tool_diff[i].name == "lspci") {
	  std::string pci_info;
	  std::string pci_class, pci_vendor, pci_device;
	  size_t pos;
	  count = 1;

      for (int j=0; tool_diff[i].newID[j] != ""; j++) {
		// Getting detailed information about pci device
		ROS_INFO("New PCI device %d", count);
		cmd = "lspci -v -mm -d " + tool_diff[i].newID[j];
		pci_info = exec(cmd.c_str());
		pos = pci_info.find("Class");
		pci_class = pci_info.substr(pos + 7, pci_info.find("\n", pos) - pos - 7);
		ROS_INFO_COND(short_output == false, "Class: \t\t%s", pci_class.c_str());
		pos = pci_info.find("Vendor");
		pci_vendor = pci_info.substr(pos + 8, pci_info.find("\n", pos) - pos - 8);
		ROS_INFO_COND(short_output == false, "Vendor: \t%s", pci_vendor.c_str());
		pos = pci_info.find("Device");
		pci_device = pci_info.substr(pos + 8, pci_info.find("\n", pos) - pos - 8);
		ROS_INFO("Device: \t%s", pci_device.c_str());
	
		decision = isGood(pci_vendor, pci_class);
		ROS_INFO("DECISION: %s", decision_to_string(decision));
		std::cout<<"\n";

		count++;	
		}
      }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Decide is device bad or good. 
//
// vendor_name - Name of vendor from USB-IF specifications
// class_name - Class from USB-IF specifications
// serial_num - Serial number of device, optional parameter, needed for Mass Storage devices
//
// returns bool value, false - device is bad, true - device is good
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool isGood(std::string vendor_name, std::string class_name, std::string serial_num) {
	if (class_name == "Mass storage" & serial_num != "") {
	  ROS_INFO_COND(short_output == false, "Device is a Mass Storage, checking serial number...");
	  if (inWhiteList("serial", serial_num)) {
		ROS_INFO_COND(short_output == false, "Serial number is allowed, device is good.");
		return true;
		}
	  else {
	   ROS_INFO_COND(short_output == false, "Serial number is not found, device is bad");
	   return false;
	   }
	  }

	else if (inWhiteList("vendor", vendor_name)) {
		ROS_INFO_COND(short_output == false, "Vendor of device is in the white list, device is good.");
		return true;
		}
	else {
	  ROS_INFO_COND(short_output == false, "Vendor of device is not in the white list, analysing further...");
	  if (inWhiteList("class", class_name)) {
		ROS_INFO_COND(short_output == false, "Class of device is in the white list, device is good.");
		return true;
		}
	  else {
	   ROS_INFO_COND(short_output == false, "Class of device is not in the white list, device is bad");
	   return false;
	   }
	 }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Check if some white list has some string in it.
//
// wl - Name of white list (vendor, class)
// s - String for search
//
// returns bool value, true - string is found, false - string is not found or there is no such white list
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool inWhiteList(std::string wl, std::string s) {
	std::string file_path = path + "white_lists/" + wl;
	int file = 0, count = 0;
	char buff[512];
	std::string list;
	memset(buff, 0, 512);
	file = open(file_path.c_str(), O_RDONLY);
	if (file < 0) // if there is no such file
	  return false;
	count = read(file, buff, 512);
	close(file);

	if (count > 0) {
	  list = buff;
	  if (list.find(s) != std::string::npos)
		return true;
	  }
	return false;	
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Execution of linux terminal command, returns output of this command
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
std::string exec(const char* cmd) {
    std::tr1::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
      if (fgets(buffer, 128, pipe.get()) != NULL)
        result += buffer;
    }
    return result;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Convert decision to string
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const char* decision_to_string(bool b) {
	if (b) return "DEVICE IS GOOD";
	else return "DEVICE IS BAD";
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Convert class code to string, from USB-IF specifications
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const char* class_to_string(uint8_t code) {
  switch (code) {
    case 0:
      return "Defined on interface level";
    case 1:
      return "Audio";
    case 2:
      return "Communications and CDC Control";
    case 3:
      return "Human Interface Device";
    case 5:
      return "Physical";
    case 6:
      return "Still Imaging";
    case 7:
      return "Printer";
    case 8:
      return "Mass storage";
    case 9:
      return "Hub";
    case 0x0a:
      return "CDC-Data";
    case 0x0b:
      return "Smart Card";
    case 0x0d:
      return "Content Security";
    case 0x0e:
      return "Video";
    case 0x0f:
      return "Personal Healthcare";
	case 0x10:
      return "Audio/Video Devices";
    case 0x11:
      return "Billboard Device";
    case 0xdc:
      return "Diagnostic Device";
    case 0xe0:
      return "Wireless Controller";
	case 0xef:
      return "Miscellaneous";
    case 0xfe:
      return "Application Specific";
    case 0xff:
      return "Vendor-specific";
    default:
      return "Unknown";
    }
}
