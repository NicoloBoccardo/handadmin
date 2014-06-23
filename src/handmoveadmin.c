// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/** 
* \file         qbmoveadmin.c
*
* \brief        Qbmoveadmin main file.
*
* \copyright    (C)  qbrobotics. All rights reserved.
*/

/**
* \mainpage     QB Move Configuration App for Unix
*
* \brief        This is an application that permits you to set up your QB Moves
*               from the command line.
*
* \version      0.1 beta 1
*
* \author       qbrobotics
*
* \date         May 26, 2012
*
* \details      This is an application that permits you to set up your QB Moves
*               from the command line.
*
*               This application can be compiled for Unix systems like Linux and
*               Mac OS X.
*
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>
#include "../../qbAPI/src/qbmove_communications.h"
#include "definitions.h"

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
#endif

// #include <unistd.h>  /* UNIX standard function definitions */
// #include <fcntl.h>   /* File control definitions */
// #include <errno.h>   /* Error number definitions */
// #include <termios.h> /* POSIX terminal control definitions */
// #include <sys/ioctl.h>



//===============================================================     structures


static const struct option longOpts[] = {
    { "id", required_argument, NULL, 'i' },
    { "PID_control_parameters", no_argument, NULL, 'k' },
    { "activate_on_startup", no_argument, NULL, 'a' },
    { "deactivate_on_startup", no_argument, NULL, 'd' },
    { "resolution", no_argument, NULL, 's' },
    { "input_mode", no_argument, NULL, 'm' },
    { "meas_multiplier", required_argument, NULL, 'u' },
    { "meas_offset", required_argument, NULL, 'o' },
    { "ping", no_argument, NULL, 'p' },
    { "restore", no_argument, NULL, 'r' },
    { "list_devices", no_argument, NULL, 'l' },
    { "serial_port", no_argument, NULL, 't' },
    { "set_limit", no_argument, NULL, 'q' },
    { "activate_limit", required_argument, NULL, 'w' },
    { "max_step", no_argument, NULL, 'b'},
    { "max_current", required_argument, NULL, 'c'},
    { "verbose", no_argument, NULL, 'v' },
    { "help", no_argument, NULL, 'h' },
    { NULL, no_argument, NULL, 0 }
};

static const char *optString = "i:kadsmu:o:prltqw:bc:vh?";

struct structglobal_args {
    int device_id;
    int flag_id;                /* -i option */
    int flag_pid_control;       /* -k option */
    int flag_activation;        /* -a option */
    int flag_deactivation;      /* -d option */
    int flag_pos_resolution;    /* -s option */
    int flag_input_mode;        /* -m option */
    int flag_meas_multiplier;   /* -u option */
    int flag_meas_offset;       /* -o option */
    int flag_ping;              /* -p option */
    int flag_restore;           /* -r option */
    int flag_list_devices;      /* -l option */
    int flag_serial_port;       /* -t option */
    int flag_set_limit;         /* -q option */
    int flag_activate_limit;    /* -w option */
    int flag_max_step;          /* -b option */
    int flag_max_current;       /* -c option */
    int flag_verbose;           /* -v option */


    unsigned char           new_id;
    unsigned char           input_mode;
    unsigned char           pos_resolution[NUM_OF_SENSORS];    
    unsigned short int      meas_offset;
    short int               measurement_offset[NUM_OF_SENSORS];
    float                   measurement_multiplier[NUM_OF_SENSORS];
    float                   meas_multiplier;    
    float                   pid_control[3];
    unsigned char           startup_activation;
    unsigned char           activate_limit;
    int                     max_step_pos;
    int                     max_step_neg;
    int16_t                 max_current;
} global_args;

//=====================================================     function definitions

/** Display program usage, and exit.
*/
void display_usage( void );

//==============================================================================
//                                                                     main loop
//==============================================================================


/** main loop
*/
int main (int argc, char **argv)
{
    int option;
    int longIndex = 0;
    FILE *file;
        
    comm_settings comm_settings_t;
    int  num_ports = 0;
    char ports[10][255];    
    char list_of_devices[255];
    char port_s[255];

    int  i = 0;
    int  aux_int;
    unsigned char aux_uchar;
    char aux_string[10000];
    // unsigned int aux_uint;

    global_args.device_id            = 0;
    global_args.flag_id              = 0;
    global_args.flag_pid_control     = 0;
    global_args.flag_activation      = 0;
    global_args.flag_deactivation    = 0;
    global_args.flag_input_mode      = 0;
    global_args.flag_pos_resolution  = 0;
    global_args.flag_meas_offset     = 0;
    global_args.flag_meas_multiplier = 0;
    global_args.flag_list_devices    = 0;
    global_args.flag_serial_port     = 0;
    global_args.flag_ping            = 0;
    global_args.flag_restore         = 0;
    global_args.flag_verbose         = 0;
    global_args.flag_max_step        = 0;
    global_args.flag_max_current     = 0;

//=======================================================     processing options


    while ((option = getopt_long( argc, argv, optString, longOpts, &longIndex )) != -1)
    {
      switch (option)
        {
        case 'i':
            sscanf(optarg,"%d", &aux_int);
            global_args.new_id = aux_int;
            global_args.flag_id = 1;
            break;
        case 'k':
            printf("Set the new values for the PID controller:\nK_p: ");
            scanf("%f", global_args.pid_control);
            printf("K_i: ");
            scanf("%f", global_args.pid_control + 1);
            printf("K_d: ");
            scanf("%f", global_args.pid_control + 2);

            printf("PID parameters: %f, %f, %f\n", global_args.pid_control[0], global_args.pid_control[1], global_args.pid_control[2]);

            global_args.flag_pid_control = 1;
            break;
        case 'a':
            global_args.flag_activation = 1;
            break;
        case 'd':
            global_args.flag_deactivation = 1;
            break;
        case 'm':
            puts("\nChoose Input Mode:\n");
            puts("[0] - Default: Motors are commanded using the communication interface.");
            puts("[1] - Encoder 3 works as a 'joystick' for the motors.");
            puts("");
            puts("--- EMG PART ---");
            puts("[2] - EMG signal is used to proportionally control hand closure.");
            puts("[3] - EMG signals are used to control opening and closing of the hand.");
            puts("");
            scanf("%d",&aux_int);
            if (aux_int < 1 || aux_int > 3) {
                aux_int = 0;
            }
            global_args.input_mode = aux_int;
            global_args.flag_input_mode = 1;
            break;
        case 's':
            puts("[0] - 360  degrees.");
            puts("[1] - 720   degrees.");
            puts("[2] - 1440  degrees.");
            puts("[3] - 2880  degrees.");
            puts("[4] - 5760  degrees.");
            puts("[5] - 11520 degrees.");
            puts("[6] - 23040 degrees.");
            puts("[7] - 46080 degrees.");
            puts("[8] - 92160 degrees.");
            puts("");

            for (i = 0; i < NUM_OF_SENSORS; i++) {
                printf("\nChoose angle resolution for sensor #%d:\n", (i + 1));
                puts("");
                scanf("%d", &aux_int);
                global_args.pos_resolution[i] = aux_int;
            }


            global_args.flag_pos_resolution  = 1;
            break;
        case 'u':
            sscanf( optarg,"%f,%f,%f",
                global_args.measurement_multiplier,
                global_args.measurement_multiplier + 1,
                global_args.measurement_multiplier + 2 );
            global_args.flag_meas_multiplier = 1;
            break;
        case 'o':
            #if NUM_OF_SENSORS == 4
                sscanf( optarg,"%hd,%hd,%hd,%hd", 
                        global_args.measurement_offset, 
                        global_args.measurement_offset + 1, 
                        global_args.measurement_offset + 2,
                        global_args.measurement_offset + 3);
            #else
                sscanf( optarg,"%hd,%hd,%hd", 
                        global_args.measurement_offset, 
                        global_args.measurement_offset + 1, 
                        global_args.measurement_offset + 2);
            #endif
            global_args.flag_meas_offset = 1;
            break;
        case 'p':
            global_args.flag_ping = 1;
            break;
        case 'r':
            global_args.flag_restore = 1;
            break;
        case 't':
            global_args.flag_serial_port = 1;
            break;            
        case 'l':
            global_args.flag_list_devices = 1;
            break;            
        case 'v':
            global_args.flag_verbose = 1;
            break;
        case 'q':
            global_args.flag_set_limit = 1;
            break;
        case 'w':
            sscanf(optarg, "%d", &aux_int);
            global_args.activate_limit = aux_int;
            global_args.flag_activate_limit = 1;
            break;
        case 'b':
            printf("Set the new values for max step:\nmax_step_pos: ");
            scanf("%d", &global_args.max_step_pos);
            printf("max_step_neg: ");
            scanf("%d", &global_args.max_step_neg);
            global_args.flag_max_step = 1;
            break;
        case 'c':
            sscanf(optarg, "%d", &aux_int);
            global_args.max_current = (int16_t)aux_int;
            global_args.flag_max_current = 1;
            break;
        case 'h':
        case '?':
        default:
            display_usage();    
            return 0;
            break;
        }
    }

    if((optind == 1) | (global_args.flag_verbose & (optind == 2)))
    {
        display_usage();    
        return 0;
    }


//======================================================     setting serial port

    if(global_args.flag_serial_port)
    {        
        num_ports = RS485listPorts(ports);
        
        if(num_ports)
        {
            puts("\nChoose serial port:\n");
            for(i = 0; i < num_ports; ++i)
            {
                printf("[%d] - %s\n\nSerial port: ", i+1, ports[i]);
                scanf("%d", &aux_int);
                
                if( aux_int && (aux_int <= num_ports) )
                {
                    file = fopen(QBMOVE_FILE, "w");
                    fprintf(file,"serialport %s",ports[aux_int-1]);
                    fclose(file);                    
                }
                else puts("Choice not available");
                
            }            
        }
        else
            puts("No serial port available.");
            
        return 0;
    }    
    
//===============================================     reading configuration file

    if(global_args.flag_verbose)
        puts("Reading configuration file.");
    
    file = fopen(QBMOVE_FILE, "r");
    fscanf(file, "serialport %s", port_s);

    if(global_args.flag_verbose)
        printf("Serial port: %s.\n",port_s);

    
//======================================================     opening serial port

    if(global_args.flag_verbose)
        puts("Connecting to serial port.");    
    
    openRS485(&comm_settings_t, port_s);
    
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
        return 0;
    }
    
    if(global_args.flag_verbose)
        puts("Serial port connected.");

//=============================================================     list devices


    if(global_args.flag_list_devices)
    {     
        aux_int = RS485ListDevices(&comm_settings_t, list_of_devices);
        
        printf("\nNumber of devices: %d \n\n", aux_int);
        puts("List of devices:");
        
        for(i = 0; i < aux_int; ++i)
        {
            printf("%d\n",list_of_devices[i]);
        }
        return 0;
    }

//========================================================     gettind device id

    if (argc - optind == 1)
    {
            sscanf(argv[optind++],"%d",&global_args.device_id);
            if(global_args.flag_verbose)
                printf("Device ID:%d\n", global_args.device_id);
    }


//=====================================================================     ping

    // If ping... then DOESN'T PROCESS OTHER COMMANDS
    if(global_args.flag_ping)
    {
        if(global_args.flag_verbose)
            puts("Pinging serial port.");
            
        if(global_args.device_id){
            commGetInfo(&comm_settings_t, global_args.device_id, INFO_ALL, aux_string);
            }
        else       
            RS485GetInfo(&comm_settings_t, aux_string);

        puts(aux_string);
        
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
         return 0;
    }

//========================================================     ...

    if ((!(argc - optind == 1)) & (global_args.flag_verbose))
        puts("No device ID was chosen. Running in broadcasting mode.");

    
//=====================================================     setting parameter ID
    
    
    // If id is changed... then DOESN'T PROCESS OTHER COMMANDS    
    if(global_args.flag_id)
    {    
        if(global_args.flag_verbose)
            printf("Changing device's id %d to %d.\n", global_args.device_id ,global_args.new_id);
        
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_ID, &global_args.new_id, 1);
        commStoreParams(&comm_settings_t, global_args.device_id);
                    
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
         return 0;
    }
    
//==================================================     setting PID parameter
    
    
    if(global_args.flag_pid_control)
    {
        if(global_args.flag_verbose) {
            printf("Changing PID control constant to:\n");
            printf("P: %f\n", global_args.pid_control[0]);
            printf("I: %f\n", global_args.pid_control[1]);
            printf("D: %f\n", global_args.pid_control[2]);
        }
        //da correggere

        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_PID_CONTROL, global_args.pid_control, 3);
        usleep(100000);
        commStoreParams(&comm_settings_t, global_args.device_id);
        
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
         return 0;
    }

//=====================================     setting parameter startup activation
    
    if(global_args.flag_activation)
    {
        if(global_args.flag_verbose)
            puts("Changing startup activation to on.");
            
        aux_uchar = 3;
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_STARTUP_ACTIVATION, &aux_uchar, 1);
        commStoreParams(&comm_settings_t, global_args.device_id);
        
        if(global_args.flag_verbose)
            puts("Closing the application.");   
        
        return 0;
    }

//=====================================     setting parameter startup activation
    
    if(global_args.flag_deactivation)
    {
        if(global_args.flag_verbose)
            puts("Changing startup activation to off.");
    
        aux_uchar = 0;
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_STARTUP_ACTIVATION, &aux_uchar, 1);
        commStoreParams(&comm_settings_t, global_args.device_id);
        
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
        return 0;
    }

//==============================================    setting parameter input mode
    
    if(global_args.flag_input_mode)
    {
        if(global_args.flag_verbose)
            puts("Changing input mode.");
            
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_INPUT_MODE, &global_args.input_mode, 1);
        commStoreParams(&comm_settings_t, global_args.device_id);
                                
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
        return 0;
    }

//====================================     setting parameter position resolution
    
    if(global_args.flag_pos_resolution)
    {
        if(global_args.flag_verbose)
            puts("Changing angle resolution.");
            
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_POS_RESOLUTION, global_args.pos_resolution, NUM_OF_SENSORS);
        commStoreParams(&comm_settings_t, global_args.device_id);
                                
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
        return 0;
    }

//====================================     setting parameter position multiplier
    
    if(global_args.flag_meas_multiplier)
    {
        if(global_args.flag_verbose)
            puts("Changing measurement position multipliers.");
            
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_MEASUREMENT_MULTIPLIER, global_args.measurement_multiplier, 3);
        commStoreParams(&comm_settings_t, global_args.device_id);
                            
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
        return 0;
    }

//========================================     setting parameter position offset
    
    if(global_args.flag_meas_offset)
    {
        if(global_args.flag_verbose)
            puts("Changing position offset.");
            
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_MEASUREMENT_OFFSET, global_args.measurement_offset, NUM_OF_SENSORS);
        commStoreParams(&comm_settings_t, global_args.device_id);
                            
        if(global_args.flag_verbose)
            puts("Closing the application.");        
        
        return 0;
    }

//=======================================     restore factory default parameters
    
    if(global_args.flag_restore)
    {
        if(global_args.flag_verbose)
            puts("Restoring factory default parameters.");

        commRestoreParams(&comm_settings_t, global_args.device_id);
                            
        if(global_args.flag_verbose)
            puts("Closing the application.");
        
        return 0;
    }

//======================================================     set position limits

    if (global_args.flag_set_limit) {
        short int limits[4];
        printf("\nInf limit 1: ");
        scanf("%hd", limits);
        printf("\nSup limit 1: ");
        scanf("%hd", limits + 1);
        printf("\nInf limit 2: ");
        scanf("%hd", limits + 2);
        printf("\nSup limit 2: ");
        scanf("%hd", limits + 3);

        printf("limits: %d, %d, %d, %d\n", limits[0], limits[1], limits[2], limits[3]);
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_POS_LIMIT, limits, 4);
        usleep(100000);
        commStoreParams(&comm_settings_t, global_args.device_id);
    }

//==================================================     activate position limit

    if (global_args.flag_activate_limit) {
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_POS_LIMIT_FLAG, &(global_args.activate_limit), 1);
        commStoreParams(&comm_settings_t, global_args.device_id);
    }

//=============================================================     set max step

    if (global_args.flag_max_step) {
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_MAX_STEP_POS, &(global_args.max_step_pos), 1);
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_MAX_STEP_NEG, &(global_args.max_step_neg), 1);
        usleep(100000);
        commStoreParams(&comm_settings_t, global_args.device_id);
    }

    if (global_args.flag_max_current) {
        commSetParam(&comm_settings_t, global_args.device_id,
            PARAM_CURRENT_LIMIT, &(global_args.max_current), 1);
        commStoreParams(&comm_settings_t, global_args.device_id);
    }

//==========================     closing serial port and closing the application

    closeRS485(&comm_settings_t);
    
    if(global_args.flag_verbose)
        puts("Closing the application.");        
    
    return 0;
}


//==============================================================================
//                                                                 dysplay usage
//==============================================================================

/** Display program usage, and exit.
*/

void display_usage( void )
{
    puts("===============================================================================");
    puts("===============================================================================");
    puts( "handmoveadmin - set up your SoftHand" );
    puts("==============================================================================="); 
    puts( "usage: handmoveadmin [id] [-i <new id> -a -vh?]");
    puts("-------------------------------------------------------------------------------"); 
    puts("Options:");
    puts("");
    puts(" -i, --id <new id>                    Change device's id.");
    puts(" -k, --PID_control_parameters         Set PID control constants.");
    puts(" -m, --input_mode                     Set input mode.");
    puts(" -a, --activate_on_startup            Set motor control active on startup.");
    puts(" -d, --deactivate_on_startup          Set motor control inactive on startup.");
    puts(" -s, --resolution                     Change sensor resolution.");
    puts(" -u, --meas_multiplier <value,...>    Set multiplier to amplify measurements.");
    puts("                                      Useful when using a joystick.");
    puts(" -o, --meas_offset <value,...>        Adds a constant offset to the measured");
    puts("                                      position using the same resolution");
    puts("                                      of the inputs.");
    puts(" -r, --restore                        Restore factory default parameters.");
    puts(" -p, --ping                           Get info on the device.");
    puts(" -l, --list_devices                   List devices connected.");
    puts(" -t, --serial_port                    Set up serial port.");
    puts(" -q, --set_limit                      Set up limits for motor maximum range.");
    puts(" -w, --activate_limit <value>         Set or reset limits position.");
    puts(" -b, --max_step                       Set maximum number of step per cycle");
    puts("                                      to slow down movements.");
    puts(" -c, --max_current <value>            Set current limit in milliampere.");
    puts("");
    puts(" -v, --verbose                        Verbose mode.");
    puts(" -h, --help                           Shows this information.");
    puts("-------------------------------------------------------------------------------"); 
    puts("Examples:");
    puts("");
    puts("  handmoveadmin -p                    Get info on whatever device is connected.");
    puts("  handmoveadmin -t                    Set up serial port.");
    puts("  handmoveadmin -l                    List devices connected.");    
    puts("  handmoveadmin 65 -i 10              Change id from 65 to 10.");    
    puts("  handmoveadmin 65 -k 0.1             Set control constant of device 65 to 0.1.");
    puts("  handmoveadmin 65 -m                 Set input mode.");    
    puts("  handmoveadmin 65 -a                 Set motors active on startup.");
    puts("  handmoveadmin 65 -d                 Set motors off on startup.");
    puts("  handmoveadmin 65 -s                 Change position resolution.");
    puts("  handmoveadmin 65 -u 2,2,2           Set measurement multipliers to 2x.");
    puts("  handmoveadmin 65 -o 50,50,50        Add an offset of 50 to all measurements.");
    puts("  handmoveadmin 65 -r                 Restore factory parameters.");
    puts("  handmoveadmin 65 -q                 Set limit for min and max position");
    puts("                                      for both motors.");
    puts("  handmoveadmin 65 -w [1 or 0]        Set or reset position limit.");
    puts("  handmoveadmin 65 -c 1000            Set current limit to 1000 mA.");
    puts("===============================================================================");
    /* ... */
    exit( EXIT_FAILURE );
}

/* [] END OF FILE */