/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include <fnet.h>
#include "stack/fnet_eth_prv.h"
#include "stack/fnet_netif_prv.h"
#include "NativeEthernet.h"
#include "utility/NativeW5100.h"

#if defined(__IMXRT1062__) || defined(ARDUINO_TEENSY41)
#include <imxrt.h>
#endif

enum LinkRecoveryState {
  LINK_RECOVERY_NORMAL = 0,
  LINK_RECOVERY_DOWN_PENDING,
  LINK_RECOVERY_SOFT_WAIT,
  LINK_RECOVERY_HARD_WAIT,
  LINK_RECOVERY_FULL_WAIT
};

static const uint32_t LINK_RECOVERY_POLL_MS = 250;
static const uint32_t LINK_DOWN_DEBOUNCE_MS = 1500;
static const uint32_t LINK_SOFT_RECOVER_WAIT_MS = 4000;
static const uint32_t LINK_HARD_RECOVER_WAIT_MS = 5000;

IPAddress EthernetClass::_dnsServerAddress;
uint8_t* EthernetClass::stack_heap_ptr = NULL;
size_t EthernetClass::stack_heap_size = 0;
ssize_t EthernetClass::socket_size = 0;
uint8_t EthernetClass::socket_num = 0;
IntervalTimer EthernetClass::_fnet_poll;
volatile boolean EthernetClass::link_status = 0;

static uint8_t recovery_mac[6];
static IPAddress recovery_ip(0, 0, 0, 0);
static IPAddress recovery_subnet(255, 255, 255, 0);
static IPAddress recovery_gateway(0, 0, 0, 0);
static IPAddress recovery_dns(0, 0, 0, 0);
static bool recovery_enabled = false;
static fnet_link_params_t stored_link_params;
static fnet_link_desc_t link_desc = 0;
static unsigned long link_dhcp_response_timeout = 4000;

static LinkRecoveryState link_recovery_state = LINK_RECOVERY_NORMAL;
static uint32_t link_recovery_phase_start_ms = 0;
static uint32_t link_recovery_last_poll_ms = 0;
static bool link_recovery_was_down = false;
DMAMEM uint8_t** EthernetClass::socket_buf_receive;
DMAMEM uint16_t* EthernetClass::socket_buf_index;
DMAMEM uint8_t** EthernetClass::socket_buf_transmit;
DMAMEM uint16_t* EthernetClass::socket_buf_len;
DMAMEM uint16_t* EthernetClass::socket_port;
DMAMEM uint8_t** EthernetClass::socket_addr;
volatile fnet_socket_t* EthernetClass::socket_ptr;

void EthernetClass::setStackHeap(uint8_t* _stack_heap_ptr, size_t _stack_heap_size){
    if(stack_heap_ptr != NULL) return;
    stack_heap_ptr = _stack_heap_ptr;
    stack_heap_size = _stack_heap_size;
}

void EthernetClass::setStackHeap(size_t _stack_heap_size){
    if(stack_heap_ptr != NULL) return;
    stack_heap_size = _stack_heap_size;
}

void EthernetClass::setSocketSize(size_t _socket_size){
    if(socket_size != 0) return;
    socket_size = _socket_size;
}

void EthernetClass::setSocketNum(uint8_t _socket_num){
    if(socket_num != 0) return;
    if(_socket_num > FNET_CFG_SOCKET_MAX) socket_num = FNET_CFG_SOCKET_MAX;
    else socket_num = _socket_num;
}

int EthernetClass::begin(uint8_t *mac, unsigned long timeout, unsigned long responseTimeout)
{
    unsigned long startMillis = millis();
    if(!fnet_netif_is_initialized(fnet_netif_get_default())){
        struct fnet_init_params     init_params;
        if(stack_heap_size == 0){
            stack_heap_size = FNET_STACK_HEAP_DEFAULT_SIZE;
        }
        if(stack_heap_ptr == NULL){
            stack_heap_ptr = new uint8_t[stack_heap_size];
        }
        if(socket_size == 0){
            socket_size = FNET_SOCKET_DEFAULT_SIZE;
        }
        if(socket_num == 0){
            socket_num = MAX_SOCK_NUM;
        }
        
        socket_buf_transmit = new uint8_t*[socket_num];
        socket_buf_receive = new uint8_t*[socket_num];
        socket_buf_len = new uint16_t[socket_num];
        socket_port = new uint16_t[socket_num];
        socket_addr = new uint8_t*[socket_num];
        socket_ptr = new fnet_socket_t[socket_num];
        socket_buf_index = new uint16_t[socket_num];
        EthernetServer::server_port = new uint16_t[socket_num];
#if FNET_CFG_TLS
        EthernetServer::tls_socket_ptr = new fnet_tls_socket_t[socket_num];
        EthernetServer::_tls = new bool[socket_num];
#endif
        
        for(uint8_t i = 0; i < socket_num; i++){
            socket_buf_transmit[i] = new uint8_t[socket_size];
            socket_buf_receive[i] = new uint8_t[socket_size];
            socket_addr[i] = new uint8_t[4];
            socket_ptr[i] = nullptr;
#if FNET_CFG_TLS
            EthernetServer::_tls[i] = false;
#endif
        }
        
        init_params.netheap_ptr = stack_heap_ptr;
        init_params.netheap_size = stack_heap_size;
        
        static const fnet_mutex_api_t teensy_mutex_api = {
            .mutex_init = teensy_mutex_init,
            .mutex_release = teensy_mutex_release,
            .mutex_lock = teensy_mutex_lock,
            .mutex_unlock = teensy_mutex_unlock,
        };
        static const fnet_timer_api_t timer_api = { //Setup millis timer
          .timer_get_ms = timer_get_ms,
          .timer_delay = 0,
        };
        /* Input parameters for FNET stack initialization */
        init_params.mutex_api = &teensy_mutex_api;
        init_params.timer_api = &timer_api;
        /* FNET Initialization */
        if (fnet_init(&init_params) != FNET_ERR) {
//          Serial.println("TCP/IP stack initialization is done.\n");
          /* You may use FNET stack API */
          /* Initialize networking interfaces using fnet_netif_init(). */
    //        Get current net interface.
          if(fnet_netif_init(FNET_CPU_ETH0_IF, mac, 6) != FNET_ERR){
//            Serial.println("netif Initialized");
            if(fnet_netif_get_default() == 0){
//              Serial.println("ERROR: Network Interface is not configurated!");
              return false;
            }
            else {
//              Serial.println("SUCCESS: Network Interface is configurated!");
              fnet_link_params_t link_params;
              link_params.netif_desc = fnet_netif_get_default();
              link_params.callback = link_callback;
              link_dhcp_response_timeout = responseTimeout;
              link_params.callback_param = &link_dhcp_response_timeout;
              stored_link_params = link_params;
              link_desc = fnet_link_init(&link_params);
              _fnet_poll.begin(fnet_poll, FNET_POLL_TIME);
            }
          }
          else {
//            Serial.println("Error:netif initialization failed.\n");
            return false;
          }
        }
        else {
//          Serial.println("Error:TCP/IP stack initialization failed.\n");
          return false;
        }
    }
    else{
//        Serial.println("Error:TCP/IP stack already initialized.");
//        return true;
    }
    
    while(!fnet_dhcp_cln_is_enabled(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()))){
        //Wait for dhcp initialization
        if(millis() - startMillis >= timeout) return false;
    }
    
    struct fnet_dhcp_cln_options current_options;
    do {//Wait for IP Address
        fnet_dhcp_cln_get_options(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()), &current_options);
        if(millis() - startMillis >= timeout) return false;
    } while (!current_options.ip_address.s_addr);
    
    return true;
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip)
{
	// Assume the DNS server will be the machine on the same network as the local IP
	// but with last octet being '1'
	IPAddress dns = ip;
	dns[3] = 1;
	begin(mac, ip, dns);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns)
{
	// Assume the gateway will be the machine on the same network as the local IP
	// but with last octet being '1'
	IPAddress gateway = ip;
	gateway[3] = 1;
	begin(mac, ip, dns, gateway);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway)
{
	IPAddress subnet(255, 255, 255, 0);
	begin(mac, ip, dns, gateway, subnet);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet)
{
    if(!fnet_netif_is_initialized(fnet_netif_get_default())){
        struct fnet_init_params     init_params;
        if(stack_heap_size == 0){
            stack_heap_size = FNET_STACK_HEAP_DEFAULT_SIZE;
        }
        if(stack_heap_ptr == NULL){
            stack_heap_ptr = new uint8_t[stack_heap_size];
        }
        if(socket_size == 0){
            socket_size = FNET_SOCKET_DEFAULT_SIZE;
        }
        if(socket_num == 0){
            socket_num = MAX_SOCK_NUM;
        }
        
        socket_buf_transmit = new uint8_t*[socket_num];
        socket_buf_receive = new uint8_t*[socket_num];
        socket_buf_len = new uint16_t[socket_num];
        socket_port = new uint16_t[socket_num];
        socket_addr = new uint8_t*[socket_num];
        socket_ptr = new fnet_socket_t[socket_num];
        socket_buf_index = new uint16_t[socket_num];
        EthernetServer::server_port = new uint16_t[socket_num];
#if FNET_CFG_TLS
        EthernetServer::tls_socket_ptr = new fnet_tls_socket_t[socket_num];
        EthernetServer::_tls = new bool[socket_num];
#endif
        
        for(uint8_t i = 0; i < socket_num; i++){
            socket_buf_transmit[i] = new uint8_t[socket_size];
            socket_buf_receive[i] = new uint8_t[socket_size];
            socket_addr[i] = new uint8_t[4];
            socket_ptr[i] = nullptr;
#if FNET_CFG_TLS
            EthernetServer::_tls[i] = false;
#endif
        }
        
        init_params.netheap_ptr = stack_heap_ptr;
        init_params.netheap_size = stack_heap_size;
    
        static const fnet_mutex_api_t teensy_mutex_api = {
            .mutex_init = teensy_mutex_init,
            .mutex_release = teensy_mutex_release,
            .mutex_lock = teensy_mutex_lock,
            .mutex_unlock = teensy_mutex_unlock,
        };
        static const fnet_timer_api_t timer_api = { //Setup millis timer
          .timer_get_ms = timer_get_ms,
          .timer_delay = 0,
        };
        /* Input parameters for FNET stack initialization */
        init_params.mutex_api = &teensy_mutex_api;
        init_params.timer_api = &timer_api;
        /* FNET Initialization */
        if (fnet_init(&init_params) != FNET_ERR) {
//          Serial.println("TCP/IP stack initialization is done.\n");
          /* You may use FNET stack API */
          /* Initialize networking interfaces using fnet_netif_init(). */
    //        Get current net interface.
          if(fnet_netif_init(FNET_CPU_ETH0_IF, mac, 6) != FNET_ERR){
//            Serial.println("netif Initialized");
            if(fnet_netif_get_default() == 0){
//              Serial.println("ERROR: Network Interface is not configurated!");
              return;
            }
            else {
//              Serial.println("SUCCESS: Network Interface is configurated!");
              fnet_link_params_t link_params;
              link_params.netif_desc = fnet_netif_get_default();
              link_params.callback = link_callback;
              link_params.callback_param = &link_dhcp_response_timeout;
              stored_link_params = link_params;
              link_desc = fnet_link_init(&link_params);
              _fnet_poll.begin(fnet_poll, FNET_POLL_TIME);
            }
          }
          else {
//            Serial.println("Error:netif initialization failed.\n");
            return;
          }
        }
        else {
//          Serial.println("Error:TCP/IP stack initialization failed.\n");
          return;
        }
    }
    else{
//        Serial.println("Error:TCP/IP stack already initialized.");
//        return;
    }
    
    fnet_dhcp_cln_release(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()));
    fnet_netif_set_ip4_addr(fnet_netif_get_default(), ip, subnet);
    fnet_netif_set_ip4_gateway(fnet_netif_get_default(), gateway);
    fnet_netif_set_ip4_dns(fnet_netif_get_default(), dns);
    
    while(!link_status){
    }
}

void EthernetClass::init(uint8_t sspin)
{
	
}

EthernetLinkStatus EthernetClass::linkStatus()
{
	switch ((uint8_t)link_status) {
		case 0:  return LinkOFF;
		case 1: return LinkON;
		default:       return Unknown;
	}
}

EthernetHardwareStatus EthernetClass::hardwareStatus()
{
    return EthernetW5500;
}

int EthernetClass::maintain()
{
	return (int)maintainLinkRecovery();
}

void EthernetClass::enableLinkRecovery(const uint8_t *mac, IPAddress ip, IPAddress subnet,
                                       IPAddress gateway, IPAddress dns)
{
  if (mac) {
    memcpy(recovery_mac, mac, 6);
  }
  recovery_ip = ip;
  recovery_subnet = subnet;
  recovery_gateway = gateway;
  recovery_dns = dns;
  recovery_enabled = true;
  link_recovery_state = LINK_RECOVERY_NORMAL;
  link_recovery_phase_start_ms = 0;
  link_recovery_was_down = false;
}


void EthernetClass::MACAddress(uint8_t *mac_address)
{
    fnet_netif_get_hw_addr(fnet_netif_get_default(), mac_address, 6);
}

IPAddress EthernetClass::localIP()
{
	return IPAddress(fnet_netif_get_ip4_addr(fnet_netif_get_default()));
}

IPAddress EthernetClass::subnetMask()
{
	return IPAddress(fnet_netif_get_ip4_subnet_mask(fnet_netif_get_default()));
}

IPAddress EthernetClass::gatewayIP()
{
	return IPAddress(fnet_netif_get_ip4_gateway(fnet_netif_get_default()));
}

IPAddress EthernetClass::dhcpServerIP()
{
    struct fnet_dhcp_cln_options current_options;
    fnet_dhcp_cln_get_options(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()), &current_options);
    
    return IPAddress(current_options.dhcp_server.s_addr);
}

void EthernetClass::setMACAddress(const uint8_t *mac_address)
{
	fnet_netif_set_hw_addr(fnet_netif_get_default(), (fnet_uint8_t*)mac_address, 6);
}

void EthernetClass::setLocalIP(const IPAddress local_ip)
{
    fnet_netif_set_ip4_addr(fnet_netif_get_default(), *const_cast<IPAddress*>(&local_ip), subnetMask());
}

void EthernetClass::setSubnetMask(const IPAddress subnet)
{
	fnet_netif_set_ip4_addr(fnet_netif_get_default(), localIP(), *const_cast<IPAddress*>(&subnet));
}

void EthernetClass::setGatewayIP(const IPAddress gateway)
{
	fnet_netif_set_ip4_gateway(fnet_netif_get_default(), *const_cast<IPAddress*>(&gateway));
}

void EthernetClass::setDnsServerIP(const IPAddress dns_server)
{
    fnet_netif_set_ip4_dns(fnet_netif_get_default(), *const_cast<IPAddress*>(&dns_server));
}

void EthernetClass::setRetransmissionTimeout(uint16_t milliseconds)
{
	//Not needed, probably
}

void EthernetClass::setRetransmissionCount(uint8_t num)
{
	//Not needed, probably
}



fnet_return_t EthernetClass::teensy_mutex_init(fnet_mutex_t *mutex) {
  return FNET_OK;
}

void EthernetClass::teensy_mutex_release(fnet_mutex_t *mutex) {
}

void EthernetClass::teensy_mutex_lock(fnet_mutex_t *mutex) {
}

void EthernetClass::teensy_mutex_unlock(fnet_mutex_t *mutex) {
}

fnet_time_t EthernetClass::timer_get_ms(void){ //Used for multi-thread version
    fnet_time_t result;
    result =  millis();
    return result;
}

static void announceLinkUp(fnet_netif_desc_t netif)
{
  fnet_netif_t *netif_ptr = (fnet_netif_t *)netif;
  if (netif_ptr && netif_ptr->netif_api && netif_ptr->netif_api->netif_change_addr_notify) {
    netif_ptr->netif_api->netif_change_addr_notify(netif_ptr);
  }
}

static int readPhyLinkDirect()
{
  fnet_netif_desc_t netif = fnet_netif_get_default();
  fnet_uint16_t data;

  if (!netif || !fnet_netif_is_initialized(netif)) {
    return 0;
  }

  if (_fnet_eth_phy_read((fnet_netif_t *)netif, FNET_ETH_MII_REG_SR, &data) != FNET_OK) {
    return -1;
  }

  return ((data & FNET_ETH_MII_REG_SR_LINK_STATUS) != 0u) ? 1 : 0;
}

static void phyHardwareReset(fnet_netif_t *netif)
{
#if defined(__IMXRT1062__) || defined(ARDUINO_TEENSY41)
  GPIO7_DR_CLEAR = (1 << 14);
  delayMicroseconds(2);
  GPIO7_DR_SET = (1 << 14);
  delayMicroseconds(5);
  _fnet_eth_phy_write(netif, 0x18, 0x0280);
  _fnet_eth_phy_write(netif, 0x17, 0x0081);
#else
  (void)netif;
#endif
}

static bool phySoftReset(fnet_netif_t *netif)
{
  if (!netif) {
    return false;
  }
  return (_fnet_eth_phy_init(netif) == FNET_OK);
}

static void reregisterLinkDetection(fnet_netif_desc_t netif)
{
  stored_link_params.netif_desc = netif;
  if (link_desc) {
    fnet_link_release(link_desc);
    link_desc = 0;
  }
  link_desc = fnet_link_init(&stored_link_params);
}

bool EthernetClass::fullNetifReinit()
{
  fnet_netif_desc_t netif;

  if (!recovery_enabled) {
    return false;
  }

  netif = fnet_netif_get_default();
  if (netif && fnet_netif_is_initialized(netif)) {
    fnet_dhcp_cln_release(fnet_dhcp_cln_get_by_netif(netif));
    fnet_netif_release(netif);
  }

  if (fnet_netif_init(FNET_CPU_ETH0_IF, recovery_mac, 6) != FNET_OK) {
    return false;
  }

  netif = fnet_netif_get_default();
  fnet_netif_set_default(netif);
  fnet_netif_set_ip4_addr(netif, recovery_ip, recovery_subnet);
  fnet_netif_set_ip4_gateway(netif, recovery_gateway);
  fnet_netif_set_ip4_dns(netif, recovery_dns);

  link_status = 0;
  reregisterLinkDetection(netif);

  fnet_netif_t *netif_ptr = (fnet_netif_t *)netif;
  if (netif_ptr) {
    netif_ptr->is_connected = FNET_FALSE;
    netif_ptr->is_connected_timestamp_ms = 0;
  }

  return true;
}

void EthernetClass::link_callback(fnet_netif_desc_t netif, fnet_bool_t connected, void *callback_param){
  link_status = connected;
  if(connected){
    announceLinkUp(netif);
    if(localIP() == IPAddress(0,0,0,0)){
       static fnet_dhcp_cln_params_t dhcp_params;
       dhcp_params.netif = netif;
       if(fnet_dhcp_cln_init(&dhcp_params)){
         fnet_dhcp_cln_set_response_timeout(fnet_dhcp_cln_get_by_netif(netif), *(unsigned long*)callback_param);
       }
    }
  }
  else{
    fnet_dhcp_cln_release(fnet_dhcp_cln_get_by_netif(netif));
  }
}

void EthernetClass::dhcp_cln_callback_updated(fnet_dhcp_cln_desc_t _dhcp_desc, fnet_netif_desc_t netif, void *p) { //Called when DHCP updates
  struct fnet_dhcp_cln_options current_options;
  fnet_dhcp_cln_get_options(_dhcp_desc, &current_options);
  
  uint8_t *ip = (uint8_t*)&current_options.ip_address.s_addr;
  Serial.print("IPAddress: ");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.println((uint8_t)*ip);

  ip = (uint8_t*)&current_options.netmask.s_addr;
  Serial.print("SubnetMask: ");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.println((uint8_t)*ip);

  ip = (uint8_t*)&current_options.gateway.s_addr;
  Serial.print("Gateway: ");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.println((uint8_t)*ip);

  ip = (uint8_t*)&current_options.dhcp_server.s_addr;
  Serial.print("DHCPServer: ");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.print((uint8_t)*ip++);
  Serial.print(".");
  Serial.println((uint8_t)*ip);

  
  Serial.print("State: ");
  Serial.println(fnet_dhcp_cln_get_state(_dhcp_desc));
  Serial.println();
  Serial.println();

  

}

EthernetLinkRecoveryResult EthernetClass::maintainLinkRecovery()
{
  uint32_t now_ms;
  int phy_link;
  fnet_netif_desc_t netif;
  fnet_netif_t *netif_ptr;
  EthernetLinkRecoveryResult result = EthernetLinkRecoveryIdle;

  if (!recovery_enabled || !fnet_netif_is_initialized(fnet_netif_get_default())) {
    return EthernetLinkRecoveryIdle;
  }

  now_ms = millis();
  if ((now_ms - link_recovery_last_poll_ms) < LINK_RECOVERY_POLL_MS) {
    return EthernetLinkRecoveryIdle;
  }
  link_recovery_last_poll_ms = now_ms;

  phy_link = readPhyLinkDirect();
  if (phy_link <= 0) {
    link_status = 0;
  }

  switch (link_recovery_state) {
    case LINK_RECOVERY_NORMAL:
      if (phy_link <= 0) {
        link_recovery_state = LINK_RECOVERY_DOWN_PENDING;
        link_recovery_phase_start_ms = now_ms;
        link_recovery_was_down = true;
      }
      break;

    case LINK_RECOVERY_DOWN_PENDING:
      if (phy_link > 0) {
        link_recovery_state = LINK_RECOVERY_NORMAL;
        if (link_recovery_was_down) {
          link_recovery_was_down = false;
          announceLinkUp(fnet_netif_get_default());
          result = EthernetLinkRecoveryLinkRestored;
        }
        break;
      }
      if ((now_ms - link_recovery_phase_start_ms) >=
          ((phy_link < 0) ? 500U : LINK_DOWN_DEBOUNCE_MS)) {
        netif = fnet_netif_get_default();
        netif_ptr = (fnet_netif_t *)netif;
        if (netif_ptr && phySoftReset(netif_ptr)) {
          link_recovery_state = LINK_RECOVERY_SOFT_WAIT;
          link_recovery_phase_start_ms = now_ms;
        }
      }
      break;

    case LINK_RECOVERY_SOFT_WAIT:
      if (phy_link > 0) {
        link_recovery_state = LINK_RECOVERY_NORMAL;
        link_recovery_was_down = false;
        link_status = 1;
        announceLinkUp(fnet_netif_get_default());
        result = EthernetLinkRecoveryLinkRestored;
        break;
      }
      if ((now_ms - link_recovery_phase_start_ms) >= LINK_SOFT_RECOVER_WAIT_MS) {
        netif = fnet_netif_get_default();
        netif_ptr = (fnet_netif_t *)netif;
        if (netif_ptr) {
          phyHardwareReset(netif_ptr);
          phySoftReset(netif_ptr);
        }
        link_recovery_state = LINK_RECOVERY_HARD_WAIT;
        link_recovery_phase_start_ms = now_ms;
      }
      break;

    case LINK_RECOVERY_HARD_WAIT:
      if (phy_link > 0) {
        link_recovery_state = LINK_RECOVERY_NORMAL;
        link_recovery_was_down = false;
        link_status = 1;
        announceLinkUp(fnet_netif_get_default());
        result = EthernetLinkRecoveryLinkRestored;
        break;
      }
      if ((now_ms - link_recovery_phase_start_ms) >= LINK_HARD_RECOVER_WAIT_MS) {
        if (fullNetifReinit()) {
          link_recovery_state = LINK_RECOVERY_FULL_WAIT;
          link_recovery_phase_start_ms = now_ms;
          result = EthernetLinkRecoveryNetifReinitialized;
        } else {
          link_recovery_state = LINK_RECOVERY_DOWN_PENDING;
          link_recovery_phase_start_ms = now_ms;
        }
      }
      break;

    case LINK_RECOVERY_FULL_WAIT:
      if (phy_link > 0) {
        link_recovery_state = LINK_RECOVERY_NORMAL;
        link_recovery_was_down = false;
        link_status = 1;
        announceLinkUp(fnet_netif_get_default());
        result = EthernetLinkRecoveryLinkRestored;
        break;
      }
      if ((now_ms - link_recovery_phase_start_ms) >= LINK_HARD_RECOVER_WAIT_MS) {
        link_recovery_state = LINK_RECOVERY_DOWN_PENDING;
        link_recovery_phase_start_ms = now_ms;
      }
      break;
  }

  return result;
}




EthernetClass Ethernet;
