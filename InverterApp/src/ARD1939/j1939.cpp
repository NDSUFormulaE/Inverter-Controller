#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>

#include "CAN_SPEC/MotorControlUnitState.h"
#include "CAN_SPEC/PGN.h"
#include "ARD1939.h"
#include "Arduino.h"

#define d49                             10

#define d50                              8

#define d31               	0x00EA00
#define d32         	0x00EAFF
#define d46                   0x00
#define d47                   0xEA
#define d48                   0x00

#define d29               	0x00EE00
#define d30         	0x00EEFF
#define d25                   0x00
#define d26            	0xEE
#define d27                   0x00
#define d28            	6

#define d33                  	0x00EE00
#define d34             	0x00FED8

#define d35                              0x00ECFF
#define d36				0x00EC00
#define d37                           0xEC
#define d38                          7

#define d39				0x00EB00
#define d40                          7

#define d41			32
#define d42			16
#define d43			17
#define d44			19
#define d45			255

#define d51			255

struct v01
{
  bool bActive;
  long v25;
};
v01 v02[MSGFILTERS];

unsigned char v03[] =
{
  (uint8_t)(NAME_IDENTITY_NUMBER & 0xFF),
  (uint8_t)((NAME_IDENTITY_NUMBER >> 8) & 0xFF),
  (uint8_t)(((NAME_MANUFACTURER_CODE << 5) & 0xFF) | (NAME_IDENTITY_NUMBER >> 16)),
  (uint8_t)(NAME_MANUFACTURER_CODE >> 3),
  (uint8_t)((NAME_FUNCTION_INSTANCE << 3) | NAME_ECU_INSTANCE),
  (uint8_t)(NAME_FUNCTION),
  (uint8_t)(NAME_VEHICLE_SYSTEM << 1),
  (uint8_t)((NAME_ARBITRARY_ADDRESS_CAPABLE << 7) | (NAME_INDUSTRY_GROUP << 4) | (NAME_VEHICLE_SYSTEM_INSTANCE))
};

#define v04                     4
long v05[] =
{
  d29,
  d34,
  d36,
  d39
};

struct v06
{
  bool v07;
  uint8_t v08;
  bool v09;
  bool v10;
  bool v11;
  uint8_t v12;
  uint8_t v13;
  uint8_t v14;
  bool v15;
  uint8_t v16;
  uint8_t v17;
};
struct v06 v18;


#if TRANSPORT_PROTOCOL == 1
  #define d01        0
  #define d02          1
  #define d03          2
  
  struct v19
  {
    uint8_t v20;
    bool v21;
    bool v22;
    bool v23;
    bool v24;
    long v25;
    uint8_t v26;
    uint8_t v27;
    uint8_t v28;
    uint8_t v29;
    uint8_t v30[J1939_MSGLEN];
    int v31;
    uint8_t v32;
  };
  v19 v33;
  v19 v34;
  
  #define d04            32
  #define d05            16
  #define d06            17
  #define d07    19
  #define d08        255
  
  #define d09        1
  #define d10    2
  #define d11     3
  
  #define d12	       0
  #define d13      1

  uint8_t v62[] = {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00};
  uint8_t v63[8];
#endif

struct v35 v38;
struct v35 v39;

#if TRANSPORT_PROTOCOL == 1
  struct v35 v40;
  struct v35 v41;
  
  struct v35 v42;
  struct v35 v43;
  struct v35 v44;
  struct v35 v45;
  struct v35 v46;
  struct v35 v47;
  struct v35 v48;
#endif

int v49;
int v50;

#if TRANSPORT_PROTOCOL == 1
  int v51;
  int v52;
  
  int v53;
  int v54;
  int v55;
  int v56;
  int v57;
  int v58;
  int v59;
#endif

#define d14      	250
#define d15       	100

#if TRANSPORT_PROTOCOL == 1
  #define d16	50
  #define d17	750
  
  #define d18     50      
  #define d19		200
  #define d20		500
  #define d21	750
  #define d22		1250
  #define d23		1250
  #define d24	1050
#endif

int v60[] = {8, 9, 10, 12, 15};
uint8_t v61;

uint8_t v64;
extern uint8_t canInit(void);
extern uint8_t canCheckError(void);
extern uint8_t canTransmit(long, unsigned char*, int);
extern uint8_t canReceive(long*, unsigned char*, int*);

struct CANVariables InverterState = {};
struct FaultEntry FaultTable[MAX_FAULTS];
bool TP_BAM_Recieved = false;
uint8_t TP_Buffer[TP_BUFFER_LENGTH];
uint8_t TP_Message_Recieved_Counter[255];
uint16_t TP_Num_Bytes = 0;
uint8_t TP_Num_Packets = 0;
uint16_t TP_PGN = 0;

uint8_t ARD1939::Init(int v80)
{
  int v65;
  f06(&v38);
  f06(&v39);
  v49 = d14 / v80;
  v50 = d15 / v80;
  v61 = 0;
  v60[0] = (int)8 / v80;
  v60[1] = (int)9 / v80;
  v60[2] = (int)10 / v80;
  v60[3] = (int)12 / v80;
  v60[4] = (int)15 / v80;
  v18.v07 = false;
  v18.v08 = false;
  v18.v09 = false;
  v18.v10 = false;
  v18.v11 = false;
  v18.v12 = SA_PREFERRED;
  v18.v13 = NULLADDRESS;
  v18.v14 = NULLADDRESS;
  v18.v15 = false;
  v18.v16 = NULLADDRESS;
  v18.v17 = NULLADDRESS;
  for(v65 = 0; v65 < MSGFILTERS; v65++)
  {
    v02[v65].bActive = false;
    v02[v65].v25 = 0;    
  }
  
#if TRANSPORT_PROTOCOL == 1
  v51 = d16 / v80;
  v52 = d17 / v80;

  v53 = d18 / v80;
  v56 = d21 / v80;
  v54 = d19 / v80;
  v55 = d20 / v80;
  v56 = d21 / v80;
  v57 = d22 / v80;
  v58 = d23 / v80;
  v59 = d24 / v80;
  f11(d12);
  f12(d12);
#endif
    
  v64 = 0;
  return canInit();
}

void ARD1939::Terminate(void)
{
  Init(SYSTEM_TIME);
}

uint8_t ARD1939::Operate(uint8_t* v70, long* v25, uint8_t* v71, int* v31, uint8_t* v67, uint8_t* v66, uint8_t* v76)
{
  uint8_t v68;
  uint8_t v82;
  int v65;
  f05();
  *v70 = f04(v25, &v71[0], v31, v67, v66, v76);
  if(*v70 == J1939_MSG_APP)
  {
    if(*v67 != v18.v17
    && *v67 != GLOBALADDRESS)
      *v70 = J1939_MSG_NETWORKDATA;
  }
  if(v18.v08 == true)
  {
    v68 = NORMALDATATRAFFIC;
    switch(*v25)
    {
      case d31:
        if(*v70 == J1939_MSG_PROTOCOL)
        {
          if(v71[0] == d27
          && v71[1] == d26
          && v71[2] == d25)
          {
            if(*v67 == GLOBALADDRESS)
            {
              Transmit(d28, d29, v18.v17, GLOBALADDRESS,
                            &v03[0], 8);
            }
            else if(*v67 == v18.v17)
            {
              Transmit(d28, d29, v18.v17, *v66,
                            &v03[0], 8);
            }
          }
        }
        break;
      
      case d29:
        if(*v66 == v18.v17)
        {
          v82 = f03(&v71[0], &v03[0]);
          switch(v82)
          {
            case 0:      
#if TRANSPORT_PROTOCOL == 1          
              f11(d12);
              f12(d12);
#endif           
              Transmit(d28, d29, NULLADDRESS, GLOBALADDRESS, &v03[0], 8);
              v18.v08 = false;
              v18.v11 = true;
              v68 = ADDRESSCLAIM_FAILED;
              break;             
            case 1:     
#if TRANSPORT_PROTOCOL == 1           
              f11(d12);
              f12(d12);
#endif           
              v18.v07 = false;
              v18.v08 = false;
              f01(*v66, &v03[0]);
              break;          
            case 2:  
              Transmit(d28, d29, v18.v17,
                            GLOBALADDRESS, &v03[0], 8);
              break;
          }
        }
        break;
      case d34:
        break;
    }
    
#if TRANSPORT_PROTOCOL == 1
    f13(*v25, v71, *v31, *v67, *v66, *v76);
    if(*v70 == J1939_MSG_NONE)
    {
    	  if(v33.v32 == true)
    	  {
    		  *v25 = v33.v25;
    		  *v31 = v33.v31;
    		  *v67 = v33.v27;
    		  *v66 = v33.v26;
    		  *v76 = d51;
    		  for(v65 = 0; v65 < v33.v31; v65++)
    			  v71[v65] = v33.v30[v65];
    		  f11(d12);
                  *v70 = J1939_MSG_APP;
    	  }
    	  else if(v34.v32 == true)
    	  {
    		  *v25 = v34.v25;
    		  *v31 = v34.v31;
    		  *v67 = v34.v27;
    		  *v66 = v34.v26;
    		  *v76 = d51;
    		  for(v65 = 0; v65 < v34.v31; v65++)
    			  v71[v65] = v34.v30[v65];
    		  f12(d12);
                  *v70 = J1939_MSG_APP;
    	  }
    }
#endif
  }
  else if(v18.v11 == true)
  {
    v68 = ADDRESSCLAIM_FAILED;
    switch(*v25)
    {
      case d31:
        if(*v70 == J1939_MSG_PROTOCOL)
        {
          if(v71[0] == d27
          && v71[1] == d26
          && v71[2] == d25)
          {
            if(*v67 == GLOBALADDRESS)
            {
              v39.v36 = v60[v61++];
              v39.v21 = true;
              if(v61 > 4)
                v61 = 0;
            }
          }
        }
        break;
      case d29:
        break;
      case d34:
        break;
    }
    if(v39.v37 == true)
    {
      f06(&v39);
       Transmit(d28, d29, NULLADDRESS, GLOBALADDRESS,
                    &v03[0], 8);
    }
  }
  else
  {
    v68 = f01(*v66, &v71[0]);
  }
  return v68;
}

uint8_t ARD1939::f01(uint8_t v16, uint8_t* v91)
{
  uint8_t v68;
  uint8_t v82;
  v68 = ADDRESSCLAIM_INPROGRESS;
  if(v18.v11 == true)
    v68 = ADDRESSCLAIM_FAILED;
  else if(v18.v08 == true)
    v68 = ADDRESSCLAIM_FINISHED;
  else if(v18.v10 == true)
  {
    if(v39.v37 == true)
    {
        f06(&v39);
        Transmit(d28, d29, v18.v16, GLOBALADDRESS,
                      &v03[0], 8);
        v38.v36 = v49;
        v38.v21 = true;
        v18.v10 = false;
    }
  }
  else
  {
    if(v18.v07 == false)
    {
      if(f02() == true)
      {
        Transmit(d28, d29, v18.v16, GLOBALADDRESS,
                      &v03[0], 8);
        v38.v36 = v49;
        v38.v21 = true;
        v18.v07 = true;
      }
      else
      {
        Transmit(d28, d29, NULLADDRESS, GLOBALADDRESS, &v03[0], 8);
        v18.v08 = false;
        v18.v11 = true;
        v68 = ADDRESSCLAIM_FAILED;
      }
    }
    else
    {
      if(v38.v37 == true)
      {
        f06(&v38);
        v18.v17 = v18.v16;
        v18.v08 = true;
        v68 = ADDRESSCLAIM_FINISHED;
      }
      else
      {
        if(canCheckError() == 1)
        {
          f06(&v38);
          if(++v64 == d49)
          {
            v18.v08 = false;
            v18.v11 = true;
            v68 = ADDRESSCLAIM_FAILED;            
          }
          else
          {
            canInit();
            v39.v36 = v60[v61++];
            v39.v21 = true;
            if(v61 > 4)
              v61 = 0;
            v18.v10 = true;
          }
        }
        else
          v64 = 0;
        if(v16 == v18.v16)
        {
          v82 = f03(&v91[0], &v03[0]);
          switch(v82)
          {
            case 0:
#if TRANSPORT_PROTOCOL == 1            
              f11(d12);
              f12(d12);
#endif           
              Transmit(d28, d29, NULLADDRESS, GLOBALADDRESS, &v03[0], 8);
              v18.v08 = false;
              v18.v11 = true;
              v68 = ADDRESSCLAIM_FAILED;
              break;       
            
              case 1:
#if TRANSPORT_PROTOCOL == 1         
              f11(d12);
              f12(d12);
#endif           
              if(f02() == true)
              {
                Transmit(d28, d29, v18.v16, GLOBALADDRESS,
                              &v03[0], 8);
                v38.v36 = v49;
                v38.v21 = true;
              }
              else
              {
                Transmit(d28, d29, NULLADDRESS, GLOBALADDRESS,
                              &v03[0], 8);
                v18.v08 = false;
                v18.v11 = true;
                v68 = ADDRESSCLAIM_FAILED;
              }
              break;

            case 2:
              Transmit(d28, d29, v18.v16, GLOBALADDRESS,
                            &v03[0], 8);
              v38.v36 = v49;
              v38.v21 = true;
              break;
          }
        }
      }
    }
  }
  return v68;
}

bool ARD1939::f02(void)
{
  bool v72;
  v72 = true;
  if(v18.v12 == NULLADDRESS)
    v18.v09 = true;
  if(v18.v09 == false)
  {
    v18.v16 = v18.v12;
    v18.v09 = true;
  }
  else
  {
    if(v18.v13 == NULLADDRESS || v18.v14 == NULLADDRESS)
    {
      v72 = false;
    }
    else
    {
      if(v18.v16 == NULLADDRESS || v18.v15 == false)
      {
        v18.v16 = v18.v13;
        v18.v15 = true;
      }
      else
      {
        if(v18.v16 < v18.v14)
          v18.v16++;
        else
          v72 = false;
      }
      if(v18.v16 == v18.v12)
      {
        if(v18.v16 < v18.v14)
          v18.v16++;
        else
          v72 = false;
      }
    }
  }
  return v72;
}

uint8_t ARD1939::f03(uint8_t* v92, uint8_t* v93)
{
  uint8_t v65;
  for(v65 = 8; v65 > 0; v65--)
  {
    if(v92[v65-1] != v93[v65-1])
    {      
      if(v92[v65-1] < v93[v65-1])
        return 1;
      else
        return 2;
    }
  }
  return 0;
}

uint8_t ARD1939::f04(long* v25, uint8_t* v71, int* v31, uint8_t* v67, uint8_t* v66, uint8_t* v76)
{
  long v78;
  long v74;
  *v25 = 0;
  *v31 = 0;
  *v67 = NULLADDRESS;
  *v66 = NULLADDRESS;
  *v76 = 0;
  if(canReceive(&v74, &v71[0], v31) == 0)
  {
    v78 = v74 & 0x1C000000;
    *v76 = (uint8_t)(v78 >> 26);
    *v25 = v74 & 0x01FFFF00;
    *v25 = *v25 >> 8;
    *v66 = (uint8_t)(v74 & 0x000000FF);
    *v67 = GLOBALADDRESS;
    if(f08(*v25) == true)
    {
      *v67 = (uint8_t)(*v25 & 0xFF);
      *v25 = *v25 & 0x01FF00;
    }
    if(f07(v25, &v71[0]) == true)
      return J1939_MSG_PROTOCOL;
    else
      return J1939_MSG_APP;
  }
  else
    return J1939_MSG_NONE;
}

uint8_t ARD1939::Transmit(uint8_t v76, long v25, uint8_t v17, uint8_t v67ess, uint8_t* v79, int v73)
{
  long v74;
  if(v73 > J1939_MSGLEN)
    return ERR;
  v74 = ((long)v76 << 26) + (v25 << 8) + (long)v17;
  if(f08(v25) == true)
    v74 = v74 | ((long)v67ess << 8);
  if(v73 > 8)
#if TRANSPORT_PROTOCOL == 1
    return f10(v76, v25, v17, v67ess, v79, v73);
#else
    return ERR;
#endif
  else
    return canTransmit(v74, v79, v73);
}

void ARD1939::f05(void)
{
  if(v38.v21 == true && v38.v37 == false)
  {
    if(--v38.v36 == 0)
      v38.v37 = true;
  }
  if(v39.v21 == true && v39.v37 == false)
    if(--v39.v36 == 0)
      v39.v37 = true;
#if TRANSPORT_PROTOCOL == 1
  if(v40.v21 == true && v40.v37 == false)
    if(--v40.v36 == 0)
    	v40.v37 = true;
  if(v41.v21 == true && v41.v37 == false)
    if(--v41.v36 == 0)
    	v41.v37 = true;
  if(v42.v21 == true && v42.v37 == false)
    if(--v42.v36 == 0)
    	v42.v37 = true;
  if(v43.v21 == true && v43.v37 == false)
    if(--v43.v36 == 0)
    	v43.v37 = true;
  if(v44.v21 == true && v44.v37 == false)
    if(--v44.v36 == 0)
    	v44.v37 = true;
  if(v45.v21 == true && v45.v37 == false)
    if(--v45.v36 == 0)
    	v45.v37 = true;
  if(v46.v21 == true && v46.v37 == false)
    if(--v46.v36 == 0)
    	v46.v37 = true;
  if(v47.v21 == true && v47.v37 == false)
    if(--v47.v36 == 0)
    	v47.v37 = true;
  if(v48.v21 == true && v48.v37 == false)
    if(--v48.v36 == 0)
    	v48.v37 = true;
#endif
}

void ARD1939::f06(struct v35* v75)
{
  v75->v36 = 0;
  v75->v21 = false;
  v75->v37 = false;
}

bool ARD1939::f07(long* v25, uint8_t* v83)
{
  bool v69;
  uint8_t v65;
  uint8_t v84;
  uint8_t v85;
  v69 = false;
  v84 = (uint8_t)((*v25 & 0x00FF00) >> 8);
  v85 = (uint8_t)(*v25 & 0x0000FF);
  switch(v84)
  {
    case d47:
      if(v85 == GLOBALADDRESS)
      {
        if(v83[0] == d27
        && v83[1] == d26
        && v83[2] == d25)
          v69 = true;
      }
      else
      {
        *v25 = *v25 & 0x00FF00;
        if(v83[0] == d27
        && v83[1] == d26
        && v83[2] == d25)
          v69 = true;
      }
      break;
  
    default:
      for(v65 = 0; v65 < v04 - 1; v65++)
      {
        if(*v25 == v05[v65])
        {
          v69 = true;
          break;
        }
      }
      break;
  }
  return v69;
}

bool ARD1939::f08(long v25)
{
  if(v25 > 0 && v25 <= 0xEFFF)
    return true;
  if(v25 > 0x10000 && v25 <= 0x1EFFF)
    return true;
  return false;
}

uint8_t ARD1939::GetSourceAddress(void)
{
   return v18.v17;
}

void ARD1939::SetPreferredAddress(uint8_t v86)
{
  v18.v12 = v86;
}

void ARD1939::SetAddressRange(uint8_t v87, uint8_t v88)
{
  v18.v13 = v87;
  v18.v14 = v88;
}

void ARD1939::SetNAME(long v81, int nManufacturerCode, uint8_t nFunctionInstance, uint8_t nECUInstance,
                  uint8_t nFunction, uint8_t nVehicleSystem, uint8_t nVehicleSystemInstance, uint8_t nIndustryGroup, uint8_t nArbitraryAddressCapable)
{
  v03[0] = (uint8_t)(v81 & 0xFF);
  v03[1] = (uint8_t)((v81 >> 8) & 0xFF);
  v03[2] = (uint8_t)(((nManufacturerCode << 5) & 0xFF) | (v81 >> 16));
  v03[3] = (uint8_t)(nManufacturerCode >> 3);
  v03[4] = (uint8_t)((nFunctionInstance << 3) | nECUInstance);
  v03[5] = (uint8_t)(nFunction);
  v03[6] = (uint8_t)(nVehicleSystem << 1);
  v03[7] = (uint8_t)((nArbitraryAddressCapable << 7) | (nIndustryGroup << 4) | (nVehicleSystemInstance));
}

uint8_t ARD1939::SetMessageFilter(long v25)
{
  uint8_t v94;
  int v65;
  v94 = ERR;
  if((v25 & 0x00FF00) == d31)
	  v25 = d31;
  if(f09(v25) == true)
    v94 = OK;
  else
  {
    for(v65 = 0; v65 < MSGFILTERS; v65++)
    {
      if(v02[v65].bActive == false)
      {
        v02[v65].bActive = true;
        v02[v65].v25 = v25;
        v94 = OK;
        break;
      }
    }
  }
  return v94;
}

void ARD1939::DeleteMessageFilter(long v25)
{
  int v65;
  if((v25 & 0x00FF00) == d31)
	  v25 = d31;
  for(v65 = 0; v65 < MSGFILTERS; v65++)
  {
    if(v02[v65].v25 == v25)
    {
      v02[v65].bActive = false;
      v02[v65].v25 = 0;
      break;
    }
  }
}

bool ARD1939::f09(long v25)
{
  bool v69;
  int v65;
  v69 = false;
  if((v25 & 0x00FF00) == d31)
	  v25 = d31;
  for(v65 = 0; v65 < MSGFILTERS; v65++)
  {
    if(v02[v65].bActive == true
    && v02[v65].v25 == v25)
    {
      v69 = true;
      break;
    }
  }
  return v69;
}

#if TRANSPORT_PROTOCOL == 1
uint8_t ARD1939::f13(long v25, uint8_t* v71, int v31, uint8_t v67, uint8_t v66, uint8_t v76)
{
  uint8_t v94;
  int nPointer;
  int v65;
  v94 = OK;
  if(v33.v20 == d03 && v33.v21 == true)
  {
      if(v33.v22 == false)
      {
        v62[0] = d04;
        v62[1] = (uint8_t)(v33.v31 & 0xFF);
        v62[2] = (uint8_t)(v33.v31 >> 8);
        v62[3] = v33.v28;
        v62[4] = 0xFF;
        v62[5] = (uint8_t)(v33.v25 & 0x0000FF);
        v62[6] = (uint8_t)((v33.v25 & 0x00FF00) >> 8);
        v62[7] = (uint8_t)(v33.v25 >> 16);
        v94 = Transmit(d38, d36, v33.v26, GLOBALADDRESS, &v62[0], 8);
        v40.v36 = v51;
        v40.v21 = true;        
        v33.v22 = true;
      }
      else
      {
        if(v40.v37 == true )
        {
          nPointer = v33.v29 * 7;
          v63[0] = ++v33.v29;
          for(v65 = 0; v65 < 7; v65++)
            v63[v65+1] = v33.v30[nPointer + v65];
          v94 = Transmit(d40, d39, v33.v26, GLOBALADDRESS, &v63[0], 8);
          if(v33.v29 == v33.v28)
          {
            f11(d12);
          }
          else
          {
            v40.v36 = v51;
            v40.v21 = true;      
            v40.v37 = false;  
          }
        }
      }
  }
  if(v25 == d36 && v71[0] == d04 && v33.v20 == d01
  && v33.v32 == false)
  {    
      v33.v25 = (((long)v71[7]) << 16) + (((long)v71[6]) << 8) + (long)v71[5];
      if(f09(v33.v25) == true)
      {
        v33.v31 = (int)v71[1] + ((int)(v71[2]) << 8);
        if(v33.v31 > J1939_MSGLEN)
        {
          f11(d12);
        }
        else
        {
          v33.v20 = d02;
          v33.v21 = true;
          v33.v26 = v66;
          v33.v27 = v67;
          v33.v28 = v71[3];
          v33.v29 = 0;
          v41.v36 = v52;
          v41.v21 = true;
        }
       }
      else 
        v33.v25 = 0;
  }
  if(v33.v20 == d02 && v41.v37 == true)
  {
    f11(d12);
  }

  if(v33.v20 == d02 && v33.v21 == true
  && v25 == d39 && v66 == v33.v26 && v67 == v33.v27)
  {
    nPointer = ((int)v71[0] - 1) * 7;
    for(v65 = 1; v65 < 8; v65++)
      v33.v30[nPointer++] = v71[v65];
    if(++v33.v29 == v33.v28)
    {
	f11(d13);
	v33.v32 = true;
    }
  }
  if(v34.v20 == d03 && v34.v21 == true
  && v25 == d36 && v71[0] == d45)
  {
    f12(d12);
  }
  
  if(v34.v20 == d03 && v34.v21 == true
  && v25 == d36 && v71[0] == d07)
  {
    f12(d12);
  }
  if(v34.v20 == d03 && v34.v21 == true)
  {
      if(v34.v23 == false)
      {
        v62[0] = d05;
        v62[1] = (uint8_t)(v34.v31 & 0xFF);
        v62[2] = (uint8_t)(v34.v31 >> 8);
        v62[3] = v34.v28;
        v62[4] = 0xFF;
        v62[5] = (uint8_t)(v34.v25 & 0x0000FF);
        v62[6] = (uint8_t)((v34.v25 & 0x00FF00) >> 8);
        v62[7] = (uint8_t)(v34.v25 >> 16);
        v94 = Transmit(d38, d36, v34.v26, v34.v27, &v62[0], 8);
        v43.v36 = v54;
        v43.v21 = true;        
        v34.v23 = true;
      }
      else
      {
        if(v43.v37 == true)
        {
          v62[0] = d08;
          v62[1] = d11;
          v62[2] = 0xFF;
          v62[3] = 0xFF;
          v62[4] = 0xFF;
          v62[5] = (uint8_t)(v34.v25 & 0x0000FF);
          v62[6] = (uint8_t)((v34.v25 & 0x00FF00) >> 8);
          v62[7] = (uint8_t)(v34.v25 >> 16);
          v94 = Transmit(d38, d36, v34.v26, v34.v27, &v62[0], 8);
          f12(d12);
        }
        if(v25 == d36 && v67 == v34.v26 && v71[0] == d06)
        {
          f06(&v43);
          v42.v36 = v53;
          v42.v21 = true;        
          v34.v24 = true;
        }
        if(v34.v24 == true && v42.v37 == true)
        {
          nPointer = v34.v29 * 7;
          v63[0] = ++v34.v29;
          for(v65 = 0; v65 < 7; v65++)
            v63[v65+1] = v34.v30[nPointer + v65];
          v94 = Transmit(d40, d39, v34.v26, v34.v27, &v63[0], 8);
          if(v34.v29 == v34.v28)
          {
            f06(&v42);
            v47.v36 = v58;
            v47.v21 = true;
          }
          else
          {
            v42.v36 = v51;
            v42.v21 = true;      
            v42.v37 = false;  
          }
        }
        if(v47.v37 == true)
        {
          v62[0] = d08;
          v62[1] = d11;
          v62[2] = 0xFF;
          v62[3] = 0xFF;
          v62[4] = 0xFF;
          v62[5] = (uint8_t)(v34.v25 & 0x0000FF);
          v62[6] = (uint8_t)((v34.v25 & 0x00FF00) >> 8);
          v62[7] = (uint8_t)(v34.v25 >> 16);
          v94 = Transmit(d38, d36, v34.v26, v34.v27, &v62[0], 8);
          f12(d12);
        }
      }
  }
  if(v25 == d36 && v67 == v18.v17 && v71[0] == d05)
  {
    int v77;
    v77 = (int)v71[1] + ((int)(v71[2]) << 8);
    if(v34.v20 != d01 || v77 > J1939_MSGLEN  || v34.v32 == true)
    {
        v62[0] = d08;
        if(v34.v31 > J1939_MSGLEN)
          v62[1] = d10;
        else
          v62[1] = d09;
        v62[2] = 0xFF;
        v62[3] = 0xFF;
        v62[4] = 0xFF;
        v62[5] = v71[5];
        v62[6] = v71[6];
        v62[7] = v71[7];
        v94 = Transmit(d38, d36, v67, v66, &v62[0], 8);
    }
    else
    {
      v34.v25 = (((long)v71[7]) << 16) + (((long)v71[6]) << 8) + (long)v71[5];
      if(f09(v34.v25) == true)
      {
        v34.v20 = d02;
        v34.v21 = true;
        v34.v24 = true;
        v34.v26 = v66;
        v34.v27 = v67;
        v34.v28 = v71[3];
        v34.v29 = 0;
        v34.v31 = (int)v71[1] + ((int)(v71[2]) << 8);
        for(v65 = 0; v65 < 8; v65++)
          v62[v65] = v71[v65];
        v62[0] = d06;
        v62[1] = v62[3];    
        v62[2] = 1;              
        v62[3] = 0xFF;          
        v94 = Transmit(d38, d36, v67, v66, &v62[0], 8);
        v45.v36 = v56;
        v45.v21 = true;
      }
      else
      {       
        v62[0] = d08;
        v62[1] = d10;
        v62[2] = 0xFF;
        v62[3] = 0xFF;
        v62[4] = 0xFF;
        v62[5] = v71[5];
        v62[6] = v71[6];
        v62[7] = v71[7];
        v94 = Transmit(d38, d36, v67, v66, &v62[0], 8);
      }
    }
  }
  if(v34.v20 == d02 && v34.v21 == true)
  {
      if(v45.v37 == true)
      {
        f12(d12);
        v62[0] = d08;
        v62[1] = d11;
        v62[2] = 0xFF;
        v62[3] = 0xFF;
        v62[4] = 0xFF;
        v62[5] = (uint8_t)(v34.v25 & 0x0000FF);
        v62[6] = (uint8_t)((v34.v25 & 0x00FF00) >> 8);
        v62[7] = (uint8_t)(v34.v25 >> 16);
        v94 = Transmit(d38, d36, v34.v27, v34.v26, &v62[0], 8);
      }
      if(v25 == d39 && v67 == v34.v27 && v66 == v34.v26)
      {
        nPointer = ((int)v71[0] - 1) * 7;
        for(v65 = 1; v65 < 8; v65++)
          v34.v30[nPointer++] = v71[v65];
        if(++v34.v29 == v34.v28)
        {
          v62[0] = d07;
          v62[1] = (uint8_t)(v34.v31 & 0x00FF);         
          v62[2] = (uint8_t)((v34.v31 & 0x00FF) >> 8);   
          v62[3] = v34.v28;
          v62[4] = 0xFF;
          v62[5] = (uint8_t)(v34.v25 & 0x0000FF);
          v62[6] = (uint8_t)((v34.v25 & 0x00FF00) >> 8);
          v62[7] = (uint8_t)(v34.v25 >> 16);
          v94 = Transmit(d38, d36, v34.v27, v34.v26, &v62[0], 8);
          f12(d13);
          v34.v32 = true;
        }
      }
  }
  return v94;
}

uint8_t ARD1939::f10(uint8_t v76, long v25, uint8_t v17, uint8_t v67ess, uint8_t* v79, int v77)
{
  uint8_t v68;
  int v65;    
  struct v19* v89;
  v68 = OK;
  if(v67ess != GLOBALADDRESS)
    v89 = &v34;
  else 
    v89 = &v33;
  if(v89->v20 != d01 || v77 > J1939_MSGLEN)
    v68 = ERR;
  else
  {
    for(v65 = 0; v65 < v77; v65++)
      v89->v30[v65] = v79[v65];
    for(v65 = v77; v65 < (v77 + 7); v65++)
    {
      if(v65 >= J1939_MSGLEN) break;
      v89->v30[v65] = 0xFF;
    }
    v89->v25 = v25;
    v89->v31 = v77;
    v89->v26 = v17;
    v89->v27 = v67ess;
    v65 = v77;
    v89->v28 = 0;
    while(v65 > 0)
    {
      v65 = v65 - 7;
      v89->v28++;
    }
    v89->v29 = 0;
    v89->v20 = d03;
    v89->v21 = true;
  }
  return v68;
}

void ARD1939::f11(uint8_t v90)
{
	if(v90 == d12)
	{
		v33.v20 = d01;
		v33.v21 = false;
		v33.v22 = false;
		v33.v25 = 0;
		v33.v26 = GLOBALADDRESS;
		v33.v27 = GLOBALADDRESS;
		v33.v28 = 0;
		v33.v29 = 0;
		v33.v31 = 0;
		v33.v32 = false;
	}
	f06(&v40);
	f06(&v41);
}

void ARD1939::f12(uint8_t v90)
{
	if(v90 == d12)
	{
		v34.v20 = d01;
		v34.v21 = false;
		v34.v23 = false;
		v34.v24 = false;
		v34.v25 = 0;
		v34.v26 = GLOBALADDRESS;
		v34.v27 = GLOBALADDRESS;
		v34.v28 = 0;
		v34.v29 = 0;
		v34.v31 = 0;
		v34.v32 = false;	
	}
	f06(&v42);
	f06(&v43);
	f06(&v44);
	f06(&v45);
	f06(&v46);
	f06(&v47);
	f06(&v48);
}

#endif



void ARD1939::CANInterpret(long* CAN_PGN, uint8_t* CAN_Message, int* CAN_MessageLen, uint8_t* CAN_DestAddr, uint8_t* CAN_SrcAddr, uint8_t* CAN_Priority){

  InverterState.Last_Message = CAN_Message;
  switch(int(*CAN_PGN)){
    case ADDRESS_CLAIM_RESPONSE:
    {
      bool found = false;
      for(int i = 0; i < NAMETABLE_LEN;i++){
        if(InverterState.Nametable[i].SA == *CAN_SrcAddr){
          break;
        }
      }
      if(found == false){
        int i;
        for(int i = 0; i < NAMETABLE_LEN;i++){
          if(InverterState.Nametable[i].SA == 0)
            break;
        }
        InverterState.Nametable[i].SA = (uint8_t)*CAN_SrcAddr;
        for(int j = 0; j< int(*CAN_MessageLen);j++){
          InverterState.Nametable[i].MESSAGE[j] = (uint8_t)*(CAN_Message + j);
        }
      }
      break;
    }
    // Shared PGN for STATUS1_RELTORQUE_SPEED, STATUS2_STATE_VOLTAGE, PROGNOSTIC1_RMS_CURRENT, PROGNOSTIC2_DIAGNOSTIC,
    // PROGNOSTIC3_DIAGNOSTIC & PROGNOSTIC5_POSITION
    case STATUS1_RELTORQUE_SPEED:
    {
      // STATUS1_RELTORQUE_SPEED
      if(CAN_Message[0] == 0x79){
        InverterState.Avg_Torque_Percent = ((CAN_Message[2] + (CAN_Message[3] << 8)) * 0.00390625) - 125.0;
        // Check if these machine speeds are different
        InverterState.Rel_Machine_Speed = ((CAN_Message[4] + (CAN_Message[5] << 8)) * 0.5) - 16000.0;        // Units: RPM
      }
      // STATUS2_STATE_VOLTAGE
      else if(CAN_Message[0] == 0x77){
        InverterState.MCU_State = CAN_Message[2];
        InverterState.DC_Bus_Voltage = (CAN_Message[3] + (CAN_Message[4] << 8)) * 0.03125;                   // Units: Volts
        InverterState.Derate_Owner = CAN_Message[5];
        InverterState.Diag_Function = CAN_Message[6] + ((CAN_Message[7] >> 3) << 8);
        InverterState.Diag_Status = CAN_Message[7] % 8; // pulls the last 3 bits from the uint8_t
      }
      // PROGNOSTIC1_RMS_CURRENT
      else if(CAN_Message[0] == 0x7A){
        InverterState.RMS_Current_Phase_A = (CAN_Message[1] + (CAN_Message[2] << 8)) * 0.0625;               // Units: Amps
        InverterState.RMS_Current_Phase_B = (CAN_Message[3] + (CAN_Message[4] << 8)) * 0.0625;               // Units: Amps
        InverterState.RMS_Current_Phase_C = (CAN_Message[5] + (CAN_Message[6] << 8)) * 0.0625;               // Units: Amps
        InverterState.Brake_Resistor_RMS_Current = CAN_Message[7];                                    // Units: Amps
      }
      // PROGNOSTIC2_DIAGNOSTIC
      else if(CAN_Message[0] == 0xF7){
        InverterState.Brake_Resistance = (CAN_Message[1] + (CAN_Message[2] << 8)) * 0.5;                     // Units: milliOhm
        InverterState.DC_Link_Capacitance = (CAN_Message[3] + (CAN_Message[4] << 8)) * 0.5;                  // Units: microFarad
        InverterState.Motor_BEMF = (CAN_Message[5] + (CAN_Message[6] << 8)) * 0.000030517578125;             // Units: Volts/RPM
        InverterState.EMI_Capacitance = CAN_Message[7] * 32;                                          // Units: nanoFarad
      }
      // PROGNOSTIC3_DIAGNOSTIC
      else if(CAN_Message[0] == 0xF8){
        InverterState.Machine_Speed_200ms_Avg = ((CAN_Message[1] + (CAN_Message[2] << 8)) * 0.5) - 16000.0;  // Units: RPM
        InverterState.Mach_Torq_Percent_200ms_Avg = ((CAN_Message[3] + (CAN_Message[4] << 8)) * 0.00390625) - 16000.0; 
      // PROGNOSTIC5_POSITION
      }
      else if(CAN_Message[0] == 0x81){
        InverterState.Stored_Pos_Offset = (CAN_Message[2] + (CAN_Message[3] << 8)) * 0.0078125;              // Units: Elec. Degrees
        InverterState.Calculated_Pos_Offset = (CAN_Message[4] + (CAN_Message[5] << 8)) * 0.0078125;          // Units: Elec. Degrees
      }
      break;
    }
    // Shared PGN for STATUS3_ABSTORQUE_SPEED, DC_LINK_PWR_STATUS && VOLTAGE_RMS1
    case STATUS3_ABSTORQUE_SPEED:
    {
      // STATUS3_ABSTORQUE_SPEED
      if(CAN_Message[0] == 0x00 && CAN_Message[1] == 0x51){
        InverterState.Avg_Abs_Torque = ((CAN_Message[2] + (CAN_Message[3] << 8)) * 0.1) - 3200.0;            // Units: Nm
        // Check if these machine speeds are different
        InverterState.Abs_Machine_Speed = (float)((long)(CAN_Message[4] + (CAN_Message[5] << 8)) / 2.0) - 16000.0;
        // Units: RPM
      }
      // DC_LINK_PWR_STATUS
      else if(CAN_Message[0] == 0x00 && CAN_Message[1] == 0x56){
        InverterState.Actual_Power = ((CAN_Message[2] + (CAN_Message[3] << 8)) * 0.001) - 32.0;
        InverterState.Max_Power_Generating = (CAN_Message[4] + (CAN_Message[5] << 8)) * 0.001;
        InverterState.Max_Power_Motoring = (CAN_Message[6] + (CAN_Message[7] << 8)) * 0.001;
      }
      // VOLTAGE_RMS1
      else if(CAN_Message[0] == 0x00 && CAN_Message[1] == 0x54){
        InverterState.RMS_Voltage_Phase_A = (CAN_Message[2] + (CAN_Message[3] << 8)) * 0.03125;               // Units: Amps
        InverterState.RMS_Voltage_Phase_B = (CAN_Message[4] + (CAN_Message[5] << 8)) * 0.03125;               // Units: Amps
        InverterState.RMS_Voltage_Phase_C = (CAN_Message[6] + (CAN_Message[7] << 8)) * 0.03125;               // Units: Amps
      }

      break;
    }
    // Shared PGN for STATUS4_TORQUE_PWRSTAGE_OVRLD, INVERTER_TEMP1_IGBT, AC_SUPPLY_STATUS & DC_LINK_PWR_CURRENT_STATUS
    case STATUS4_TORQUE_PWRSTAGE_OVRLD:
    {
      // STATUS4_TORQUE_PWRSTAGE_OVRLD
      if(CAN_Message[0] == 0x32){
      InverterState.Neg_Torque_Available = ((CAN_Message[2] + (CAN_Message[3] << 8)) * 0.1) - 3200.0;        // Units: Nm
      InverterState.Pos_Torque_Available = ((CAN_Message[2] + (CAN_Message[3] << 8)) * 0.1) - 3200.0;        // Units: Nm
      // Power Stage Status Values
      // 0 = Outputs Off
      // 1 = Normal Switching
      // 2 = High Side Three Phase Short
      // 3 = Low Side Three Phase Short
      InverterState.Power_Stage_Status = CAN_Message[6];
      InverterState.Overload_Percent = CAN_Message[7] * 0.5;
      }
      // INVERTER_TEMP1_IGBT
      else if(CAN_Message[0] == 0x90){
        InverterState.IGBT1_Temp = CAN_Message[1] - 40;                                                // Units: Degrees Celcius
        InverterState.IGBT2_Temp = CAN_Message[2] - 40;                                                // Units: Degrees Celcius
        InverterState.IGBT3_Temp = CAN_Message[3] - 40;                                                // Units: Degrees Celcius
        InverterState.IGBT4_Temp = CAN_Message[4] - 40;                                                // Units: Degrees Celcius
        InverterState.IGBT5_Temp = CAN_Message[5] - 40;                                                // Units: Degrees Celcius
        InverterState.IGBT6_Temp = CAN_Message[6] - 40;                                                // Units: Degrees Celcius
        InverterState.Brake_Chopper_IGBT_Temp = CAN_Message[7] - 40;                                   // Units: Degrees Celcius
      }
      // AC_SUPPLY_STATUS
      else if(CAN_Message[0] == 0x31){
        InverterState.AC_Voltage_Output = CAN_Message[2] + (CAN_Message[3] << 8);                         // Units: Vrms
        InverterState.AC_Frequency = CAN_Message[4] + (CAN_Message[5] << 8);                              // Units: Hz
        InverterState.AC_Voltage_Desired = CAN_Message[6] + (CAN_Message[7] << 8);                        // Units: Vrms
      }
      // DC_LINK_PWR_CURRENT_STATUS
      else if(CAN_Message[0] == 0x36){
        InverterState.Actual_Current = ((CAN_Message[2] + (CAN_Message[3] << 8)) * 0.001) - 32.0;
        InverterState.Max_Current_Generating = (CAN_Message[4] + (CAN_Message[5] << 8)) * 0.001;
        InverterState.Max_Current_Motoring = (CAN_Message[6] + (CAN_Message[7] << 8)) * 0.001;
      }

      break;
    }
    case INVERTER_TEMP2_MACHINE:
    {
      if(CAN_Message[0] == 0xE4){
        InverterState.Motor_Temp_1 = CAN_Message[1] - 40;                                              // Units: degrees Celcius
        InverterState.Motor_Temp_2 = CAN_Message[2] - 40;                                              // Units: degrees Celcius
        InverterState.Motor_Temp_3 = CAN_Message[3] - 40;                                              // Units: degrees Celcius

        InverterState.Brake_Resistor_Temp = CAN_Message[5] - 40;                                       // Units: degrees Celcius
        InverterState.Control_Board_Temp = CAN_Message[6] - 40;                                        // Units: degrees Celcius
        InverterState.Inverter_Coolant_Temp = CAN_Message[7] - 40;                                     // Units: degrees Celcius
      }
      break;
    }
    case DM1:
    {
      InverterState.Protect_Lamp_Status = CAN_Message[0] >> 6;
      InverterState.Amber_Warning_Lamp_Status = (CAN_Message[0] >> 4) % 4;
      InverterState.Red_Stop_Lamp_Status = (CAN_Message[0] >> 2) % 4;
      InverterState.Multi_Indicator_Lamp_Status = CAN_Message[0] % 4;

      InverterState.Flash_Protect_Lamp_Status = CAN_Message[1] >> 6;
      InverterState.Flash_Amber_Warning_Lamp_Status = (CAN_Message[1] >> 4) % 4;
      InverterState.Flash_Red_Stop_Lamp_Status = (CAN_Message[1] >> 2) % 4;
      InverterState.Flash_Multi_Indicator_Lamp_Status = CAN_Message[1] % 4;

      unsigned long SPN_Top = (unsigned long)(CAN_Message[4] >> 5);
      unsigned long SPN_Mid = (unsigned long)CAN_Message[3];
      unsigned long SPN_Bottom = CAN_Message[2];
      unsigned long SPN = ((SPN_Top << 16) + (SPN_Mid << 8) + SPN_Bottom);
      uint8_t FMI = CAN_Message[4] & 0b11111;
      uint8_t Occ = CAN_Message[5] & 0b1111111;
      // Serial.print("FAULT CAN 3: ");
      // Serial.println(CAN_Message[3]);
      // Serial.print("FAULT CAN 4: ");
      // Serial.println(CAN_Message[4]);
      // Serial.print("FAULT CAN 5: ");
      // Serial.println(CAN_Message[5]);
      UpdateAddFault(SPN, FMI, Occ);
      break;
    }

    case TP_BAM:
    {
      TP_BAM_Recieved = true;
      TP_Num_Bytes = CAN_Message[1] + (CAN_Message[2] << 8);
      TP_Num_Packets = CAN_Message[3];
      TP_PGN = CAN_Message[5] + (CAN_Message[6] << 8);
      for(int i = 0; i < TP_Num_Packets; i++)
      {
        if(TP_Message_Recieved_Counter[i] == 1)
        {
          TP_Message_Recieved_Counter[i] = 0;
        }
      }
      for(int i = 0; i < TP_Num_Bytes; i++){
        if(TP_Buffer[i] != 0xFF){
          TP_Buffer[i] = 0xFF;
        }
      }
      // Serial.print("TP_BAM Recieved PGN: "); 
      // Serial.print(TP_PGN);
      // Serial.print(" Num_Bytes: ");
      // Serial.println(TP_Num_Bytes);
      break;
    }

    case TP_DATA:
    {
      uint8_t message_index = CAN_Message[0] - 1;
      uint16_t message_offset = 7 * message_index;
      TP_Message_Recieved_Counter[message_index] = 1;
      for(int i = 0; i < 7; i++)
      {
        TP_Buffer[message_offset + i] = CAN_Message[i+1];
      }
      if(TPMessageRecived(TP_Num_Packets))
      {
        DecodeTransportProtocol();
      }
      break;
    }
  }
}

bool ARD1939::TPMessageRecived(uint8_t num_packets)
  /**
  * Verifies all packets in a TP message have been recieved.
  * 
  * Parameters:
  *     num_packets (uint8_t): The number of packets in the transport
  *                            protocol message
  * Returns:
  *     (bool): false if any message has failed to be recieved, else true.
  **/
{
  for (int i = 0; i < num_packets; i++)
  {
    if (TP_Message_Recieved_Counter[i] != 1)
    {
      return false;
    }
  }
  return true;
}

void ARD1939::DecodeTransportProtocol()
{
    /**
    * Reads through the message stored in TP_Buffer and adds active faults to the fault table.
    * 
    * Parameters:
    *     none
    * Returns:
    *     none
    **/
  if(TP_PGN == DM1){
    InverterState.Protect_Lamp_Status = TP_Buffer[0] >> 6;
    InverterState.Amber_Warning_Lamp_Status = (TP_Buffer[0] >> 4) % 4;
    InverterState.Red_Stop_Lamp_Status = (TP_Buffer[0] >> 2) % 4;
    InverterState.Multi_Indicator_Lamp_Status = TP_Buffer[0] % 4;

    InverterState.Flash_Protect_Lamp_Status = TP_Buffer[1] >> 6;
    InverterState.Flash_Amber_Warning_Lamp_Status = (TP_Buffer[1] >> 4) % 4;
    InverterState.Flash_Red_Stop_Lamp_Status = (TP_Buffer[1] >> 2) % 4;
    InverterState.Flash_Multi_Indicator_Lamp_Status = TP_Buffer[1] % 4;

    int Num_DM1s = (TP_Num_Bytes - 2)/4;
    for(int i = 0; i < Num_DM1s; i++)
    {
      unsigned long TP_SPN_Top = (unsigned long)(TP_Buffer[(4*i) + 4] >> 5);
      unsigned long TP_SPN_Mid = (unsigned long)TP_Buffer[(4*i) + 3];
      unsigned long TP_SPN_Bottom = TP_Buffer[(4*i) + 2];
      unsigned long SPN = ((TP_SPN_Top << 16) + (TP_SPN_Mid << 8) + TP_SPN_Bottom);
      uint8_t FMI = TP_Buffer[(4*i) + 4] & 0b11111;
      uint8_t Occ = TP_Buffer[(4*i) + 5] & 0b1111111;
      UpdateAddFault(SPN, FMI, Occ);
    }
  } 
}

int ARD1939::FirstFreeInFaultArray()
{
    /**
    * Iterates over the FaultTable array until it finds the first open spot in the array.
    * 
    * Parameters:
    *     none
    * Returns:
    *     i (int): index of the first free object in array, -1 if full.
    **/
    for (int i = 0; i < MAX_FAULTS; i++)
    {
        if (FaultTable[i].active == 0)
        {
            return i;
        }
    }
    return -1;
}

bool ARD1939::isFaultTableClear()
{
    /**
    * Iterates over the FaultTable array and checks if all faults are inactive.
    * 
    * Parameters:
    *     none
    * Returns:
    *     (bool): true if no faults in array, else false.
    **/
    for (FaultEntry fault : FaultTable)
    {
        if (fault.active)
        {
            return false;
        }
    }
    return true;
}

int ARD1939::isFaultInArray(unsigned long SPN, uint8_t FMI)
{
    /**
    * Iterates over the FaultTable array and checks if active fault
    * matching SPN and FMI in the is in the array
    * 
    * Parameters:
    *     SPN       (uint16_t): Suspect Parameter Number of Fault
    *     FMI        (uint8_t): Failure mode identifier
    * Returns:
    *     index          (int): Index of fault if exists and is active.
    *                           else -1.
    **/
    for (int i = 0; i < MAX_FAULTS; i++)
    {
        if((FaultTable[i].active != 0) && (FaultTable[i].SPN == SPN && FaultTable[i].FMI == FMI))
        {
            return i;
        }
    }
    return -1;
}

int ARD1939::AddNewFault(unsigned long SPN, uint8_t FMI, uint8_t Occurance)
{
    /**
    * Configure a new FaultEntry in array.
    *
    * Parameters:
    *     SPN       (uint16_t): Suspect Parameter Number of Fault
    *     FMI        (uint8_t): Failure mode identifier
    *     OC         (uint8_t): Number of fault occurances 
    *     CM         (uint8_t): SPN conversion method
    * Returns:
    *     firstFree      (int): Index of the newly added FaultEntry
    **/
    int firstFree = FirstFreeInFaultArray();
    if (firstFree != -1)
    {
        FaultTable[firstFree].SPN = SPN;
        FaultTable[firstFree].FMI = FMI;
        FaultTable[firstFree].OC = Occurance;
        FaultTable[firstFree].active = 1;
    }
    return firstFree;
}

int ARD1939::UpdateAddFault(unsigned long SPN, uint8_t FMI, uint8_t Occurance)
{
    /**
    * Configure a new FaultEntry in array, if it already exists update the occurance.
    *
    * Parameters:
    *     SPN       (uint16_t): Suspect Parameter Number of Fault
    *     FMI        (uint8_t): Failure mode identifier
    *     OC         (uint8_t): Number of fault occurances 
    *     CM         (uint8_t): SPN conversion method
    * Returns:
    *     firstFree      (int): Index of the newly added FaultEntry
    **/
    int faultIndex = isFaultInArray(SPN, FMI);
    char sString[30];
    if(faultIndex > -1)
    {
        if(FaultTable[faultIndex].OC < Occurance)
        {
          FaultTable[faultIndex].OC = Occurance;
          Serial.print("FAULT UPDATE - SPN: ");
          sprintf(sString, "%lu ", SPN);
          Serial.print(sString);
          Serial.print(" FMI: ");
          Serial.print(FMI);
          Serial.print(" Occ: ");
          Serial.println(Occurance);
        } 
    }
    else
    {
        AddNewFault(SPN, FMI, Occurance);
        Serial.print("FAULT NEW - SPN: ");
        sprintf(sString, "%lu ", SPN);
        Serial.print(sString);
        Serial.print(" FMI: ");
        Serial.print(FMI);
        Serial.print(" Occ: ");
        Serial.println(Occurance);
    }
    return faultIndex;
}

void ARD1939::ClearFaultTable(void)
{
    /**
    * Iterate over the FaultTable and mark all as inactive
    *
    * Parameters:
    *     none
    * Returns:
    *     none
    **/
  for(int i = 0; i < MAX_FAULTS; i++)
  {
    FaultTable[i].active = 0;
  }
}

void ARD1939::ClearFaults(void)
{
    /**
    * Clear the internal FaultTable and send a DM3 + DM11 message.
    *
    * Parameters:
    *     none
    * Returns:
    *     none
    **/
  Serial.println("Clearing Bus Faults");
  ARD1939::ClearFaultTable();
  uint8_t priority = 6;
  uint8_t clear_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ARD1939::Transmit(priority, DM3, ARD1939::GetSourceAddress(), 0xA2, &clear_data[0], 8);
  ARD1939::Transmit(priority, DM11, ARD1939::GetSourceAddress(), 0xA2, &clear_data[0], 8);
}

bool ARD1939::CheckValidState(void)
{
    /**
    * Checks the Current Inverter state and checks if we are in a non-fault state
    *
    * Parameters:
    *     none
    * Returns:
    *     (bool): true if in a valid state, else false.
    **/
  switch(InverterState.MCU_State){
    case MCU_STDBY: 
    case MCU_IGNIT_READY: 
    case MCU_PWR_READY: 
    case MCU_DRIVE_READY:
    case MCU_NORM_OPS: return true; break;
    default: return false; break;
  }
}
