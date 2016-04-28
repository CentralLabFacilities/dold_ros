/***
  * dold driver
 */

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>

#include <dold_msgs/DoldState.h>
#include <dold_msgs/DoldStates.h>

#define NUM_BUTTONS 4

#define HEADER_OFFSET 5 // dold:
#define PROTOCOL_LENGTH 6 //HEADER_OFFSET + 1 for button number

using std::hex;
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();
	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
	}
}

void print_usage()
{
	cerr << "Usage: dold_driver {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int run(int argc, char **argv)
{
  ros::init(argc, argv, "teensy_tactile");
  ros::NodeHandle nh;
  
  dold_msgs::DoldStates dold_msg;
  dold_msg.header.frame_id="";

  ros::Publisher pub = nh.advertise<dold_msgs::DoldStates>("dold_driver/state",2);
  
  if(argc < 2) {
	  print_usage();
    return 0;
  }

  // Argument 1 is the serial port or enumerate flag
  string port(argv[1]);

  if( port == "-e" ) {
	  enumerate_ports();
	  return 0;
  }
  else if( argc < 3 ) {
	  print_usage();
	  return 1;
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  //cout << "Is the serial port open?";
  if(!my_serial.isOpen())
  {
    cout << " Failed to open serial port." << endl;
    return -1;
  }

  cout<<"Waiting for header:" << endl;
  uint8_t buffer[PROTOCOL_LENGTH + NUM_BUTTONS];
  bool found_header=false;
  size_t bytes_read;
  //sync to find the beginning of the sequence
  try {
     bytes_read = my_serial.read(buffer, 1);
  }
  catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
  //cout << "Read " << bytes_read << " bytes"<<endl;
  //cout<<">" << buffer[0];
  //for (size_t i=0 ; i<bytes_read ; ++i)
     // cout <<(char)buffer[i] <<", ";


  size_t offset=0;
  int count=0;
  unsigned int num_buttons=0;
  while (!found_header && count < PROTOCOL_LENGTH + NUM_BUTTONS) {
    count++;
    if (buffer[0]!='D')
    {
      cout<<"." << buffer[0];
    }
    else
    {
      // already read one byte, read the reminder of the protocol
      bytes_read = my_serial.read(buffer, PROTOCOL_LENGTH-1);
      if (bytes_read == PROTOCOL_LENGTH-1)
      {
        if (buffer[HEADER_OFFSET-2]==':')
        {
          found_header=true;
          num_buttons = (unsigned int)buffer[HEADER_OFFSET-1];
          cout<<"found header with "<<num_buttons << " buttons" << endl;
          if (num_buttons > 0)
          {
            //clear up the remaining data
            bytes_read = my_serial.read(buffer, num_buttons);
            if(num_buttons > NUM_BUTTONS)
            {
              cout<<"Too many buttons in the protocol "<< num_buttons << " instead of :" << NUM_BUTTONS << endl;
              my_serial.close();
              return -1;
            }
          }
          break;
        }
        else
        {
          cout<<"found "<< hex << buffer[HEADER_OFFSET-2] << " instead of :" << endl;
        }
      }
      else
      {
        cout << "Read " << bytes_read << " bytes, awaited " << PROTOCOL_LENGTH-1 << endl;
      }
    }
    bytes_read = my_serial.read(buffer, 1);
    //cout << "Read " << bytes_read << " bytes"<<endl;
    //cout<<"." << buffer[0];
  }
  
  dold_msg.inputs.resize(NUM_BUTTONS);
  for (size_t i = 0; i < NUM_BUTTONS; ++i)
  {
    std::stringstream ss;
    ss << "B" << i;
    dold_msg.inputs[i].name = ss.str();
    dold_msg.inputs[i].type = dold_msgs::DoldState::BUTTON;
    ss.str("");
  }
  
  while (ros::ok() && found_header) {
    bytes_read = my_serial.read(buffer, PROTOCOL_LENGTH);
    num_buttons =  (unsigned int)buffer[HEADER_OFFSET];
    //cout << "Read "  << bytes_read << " bytes and found " << num_buttons << "buttons" <<endl;
    
    if(num_buttons > 0 and num_buttons <= NUM_BUTTONS){
      // read the data now
      bytes_read = my_serial.read(buffer, num_buttons);
      if (bytes_read == num_buttons)
      {
        for (size_t i=0; i<num_buttons; ++i)
        {
          dold_msg.inputs[i].state = buffer[i];
          //cout << "button " << i << " val "<< (int)buffer[i] << endl;
        }

        dold_msg.header.stamp=ros::Time::now();
        pub.publish(dold_msg);
        ros::spinOnce();
      }
      else
        cout << "Read " << bytes_read << " bytes"<<endl;
    }
  }
  if(!found_header)
    cout<<"Did not find the header. Is the device connected and using the correct protocol ?"<<endl;
  my_serial.close();
  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
