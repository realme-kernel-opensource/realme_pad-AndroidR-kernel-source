#ifndef KD_IMGSENSORINFO_H
#define KD_IMGSENSORINFO_H

/*****************************************
     CXXXXXX_SENSOR_ID_XXXX 0x0000 000
     CXXXXXX_SENSOR_DRVVANE_XXXX "name"
     C    ->   C  is mean Camera
     XXX  ->  XXX is mean module  Code
     XXX  ->  XXX is mean sensor  Code
     XXXX -> XXXX is mean project Code
     0000 -> 0000 is mean sensor  id
     000  ->  000 is mean module  id
******************************************/
/*-------------------SENSOR ID-----------------------*/
// main sensor id
        /* hi846 */
#define HI846A_SENSOR_ID                        0x4608
#define HI846B_SENSOR_ID                        0x4609
#define SC800CSA_SENSOR_ID                      0xd126
#define SC800CSB_SENSOR_ID                      0xd127
#define GC08A3A_SENSOR_ID                       0x08a3
#define GC08A3B_SENSOR_ID                       0x08a4

    // sub sensor id
    
    // main2 sensor id
    
    // sub2 sensor id
    
    // main3 sensor id
    
    
    /*-----------------SENSOR NAME---------------------*/
    // main sensor name
#define SENSOR_DRVNAME_HI846A_MIPI_RAW      "hi846a_mipi_raw"
#define SENSOR_DRVNAME_HI846B_MIPI_RAW      "hi846b_mipi_raw"
#define SENSOR_DRVNAME_SC800CSA_MIPI_RAW        "sc800csa_mipi_raw"
#define SENSOR_DRVNAME_SC800CSB_MIPI_RAW        "sc800csb_mipi_raw"
#define SENSOR_DRVNAME_GC08A3A_MIPI_RAW         "gc08a3a_mipi_raw"
#define SENSOR_DRVNAME_GC08A3B_MIPI_RAW         "gc08a3b_mipi_raw"

// sub sensor name

// main2 sensor name

// sub2 sensor name

// main3 sensor name
#endif
