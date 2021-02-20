/*
 *  vdr-plugin-dvbapi - softcam dvbapi plugin for VDR
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/ioctl.h>
#include <linux/dvb/ca.h>
#include <netinet/tcp.h>
#include <poll.h>
#include "SocketHandler.h"
#include "Log.h"

#define SOCKET_CHECK_INTERVAL   3000

SocketHandler *SockHandler = NULL;

SocketHandler::~SocketHandler()
{
  Cancel(3);
  CloseConnection();
}

SocketHandler::SocketHandler()
:cThread("Socket Handler")
{
  DEBUGLOG("%s", __FUNCTION__);
  sock = 0;
  protocol_version = 0;
  Start();
}

void SocketHandler::OpenConnection()
{
  cMutexLock lock(&mutex);

  // connecting via TCP socket to OSCam
  struct addrinfo hints, *servinfo, *p;
  int rv;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  if ((rv = getaddrinfo(OSCamHost, itoa(OSCamPort), &hints, &servinfo)) != 0)
  {
    ERRORLOG("getaddrinfo error: %s", gai_strerror(rv));
    return;
  }

  // loop through all the results and connect to the first we can
  for (p = servinfo; p != NULL; p = p->ai_next)
  {
    int sockfd;
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1)
    {
        ERRORLOG("%s: socket error: %s", __FUNCTION__, strerror(errno));
        continue;
    }
    if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1)
    {
        close(sockfd);
        ERRORLOG("%s: connect error: %s", __FUNCTION__, strerror(errno));
        continue;
    }
    sock = sockfd;
    break; // if we get here, we must have connected successfully
  }

  if (p == NULL)
  {
    // looped off the end of the list with no connection
    ERRORLOG("Cannot connect to OSCam. Check your configuration and firewall settings.");
    sock = 0;
  }

  freeaddrinfo(servinfo); // all done with this structure

  if (sock == 0)
    return;

  int val = 1;
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (int*)&val, sizeof(val));

  DEBUGLOG("created socket with socket_fd=%d", sock);
}

void SocketHandler::CloseConnection()
{
  if (sock > 0)
  {
    close(sock);
    sock = 0;
    filter->StopAllFilters();
  }
}

void SocketHandler::Write(unsigned char *data, int len)
{
  DEBUGLOG("%s, sock=%d", __FUNCTION__, sock);
  if (sock == 0)
    return;

  if (!WriteData(data, len, 500))
    CloseConnection();
}

bool SocketHandler::Read(unsigned char *data, int len)
{
  DEBUGLOG("%s, sock=%d", __FUNCTION__, sock);
  if (sock < 0)
    return false;

  int rc = ReadData(data, len, 1000);

  if (rc == ECONNRESET)
  {
    ERRORLOG("%s: connection reset", __FUNCTION__);
    CloseConnection();
    return false;
  }

  return (rc == 0);
}

void SocketHandler::SendFilterData(unsigned char demux_id, unsigned char filter_num, unsigned char *data, int len)
{
  unsigned char buff[6 + len];

  uint32_t req = htonl(DVBAPI_FILTER_DATA);               //type of request
  memcpy(&buff[0], &req, 4);
  buff[4] = demux_id;                                     //demux
  buff[5] = filter_num;                                   //filter
  memcpy(buff+6, data, len);                              //copy filter data
  SockHandler->Write(buff, sizeof(buff));
}

void SocketHandler::SendClientInfo()
{
  int len = sizeof(INFO_VERSION) - 1;                     //ignoring null termination
  unsigned char buff[7 + len];

  uint32_t req = htonl(DVBAPI_CLIENT_INFO);               //type of request
  memcpy(&buff[0], &req, 4);
  int16_t proto_version = htons(DVBAPI_PROTOCOL_VERSION); //supported protocol version
  memcpy(&buff[4], &proto_version, 2);
  buff[6] = len;
  memcpy(&buff[7], &INFO_VERSION, len);                   //copy info string
  SockHandler->Write(buff, sizeof(buff));
}

void SocketHandler::SendStopDescrambling()
{
  DEBUGLOG("%s", __FUNCTION__);
  unsigned char buff[8];

  buff[0] = 0x9F;
  buff[1] = 0x80;
  buff[2] = 0x3F;
  buff[3] = 0x04;

  buff[4] = 0x83;
  buff[5] = 0x02;
  buff[6] = 0x00;
  buff[7] = 0xFF;             //wildcard demux id

  Write(buff, 8);
}

void SocketHandler::Action(void)
{
  DEBUGLOG("%s", __FUNCTION__);
  unsigned char buff[262];
  uint32_t *request;
  uint8_t adapter_index;

  while (Running())
  {
    if (sock == 0 && capmt && !capmt->Empty())
    {
      DEBUGLOG("OSCam not connected, (re)connecting...");
      OpenConnection();
      if (sock > 0)
      {
        DEBUGLOG("Successfully (re)connected to OSCam");
        SendClientInfo();
        capmt->SendAll();
      }
    }

    if (sock == 0)
    {
      cCondWait::SleepMs(20);
      continue;
    }

    // request
    int status = ReadData(buff, sizeof(int), 100);

    if (status == ECONNRESET)
    {
      CloseConnection();
      continue;
    }

    if (status == ETIMEDOUT || status != 0)
    {
      continue;
    }

    request = (uint32_t *) &buff;

    if (ntohl(*request) != DVBAPI_SERVER_INFO)
    {
      // first byte -> adapter_index
      if (!Read(&adapter_index, 1)) {
        continue;
      }
      adapter_index -= AdapterIndexOffset;
    }

    bool rc = true;

    *request = ntohl(*request);
    if (*request == CA_SET_PID)
      rc = Read(buff+4, sizeof(ca_pid_t));
    else if (*request == CA_SET_DESCR)
      rc = Read(buff+4, sizeof(ca_descr_t));
    else if (*request == CA_SET_DESCR_AES)
      rc = Read(buff+4, sizeof(ca_descr_aes_t));
    else if (*request == CA_SET_DESCR_MODE)
      rc = Read(buff+4, sizeof(ca_descr_mode_t));
    else if (*request == CA_SET_DESCR_DATA)
      rc = Read(buff+4, sizeof(ca_descr_data_t));
    else if (*request == DMX_SET_FILTER)
      rc = Read(buff+4, sizeof(struct dmx_sct_filter_params));
    else if (*request == DMX_STOP)
      rc = Read(buff+4, 2 + 2);
    else if (*request == DVBAPI_SERVER_INFO)
    {
      unsigned char len;
      rc &= Read(buff+4, 2);             //proto version
      rc &= Read(&len, 1);               //string length
      rc &= Read(buff+6, len);
      buff[6+len] = 0;                                 //terminate the string
    }
    else if (*request == DVBAPI_ECM_INFO)
      rc = Read(buff+4, 14);            //read ECM info const len header only
    else
    {
      ERRORLOG("%s: read failed unknown command: %08x", __FUNCTION__, *request);
      CloseConnection(); // no chance to recover in this case -> reconnect
      continue;
    }

    if (!rc)
    {
      cCondWait::SleepMs(20);
      continue;
    }

    if (*request == CA_SET_PID)
    {
      DEBUGLOG("%s: Got CA_SET_PID request, adapter_index=%d", __FUNCTION__, adapter_index);
      memcpy(&ca_pid, &buff[sizeof(int)], sizeof(ca_pid_t));
      ca_pid.pid = ntohl(ca_pid.pid);
      ca_pid.index = ntohl(ca_pid.index);
      decsa->SetCaPid(adapter_index, &ca_pid);
    }
    else if (*request == CA_SET_DESCR)
    {
      memcpy(&ca_descr, &buff[sizeof(int)], sizeof(ca_descr_t));
      ca_descr.index = ntohl(ca_descr.index);
      ca_descr.parity = ntohl(ca_descr.parity);
      decsa->SetAes(ca_descr_aes.index, false);
      decsa->SetDescr(&ca_descr, false);
      DEBUGLOG("%s: Got CA_SET_DESCR request, adapter_index=%d, index=%x", __FUNCTION__, adapter_index, ca_descr.index);
    }
    else if (*request == CA_SET_DESCR_AES)
    {
      memcpy(&ca_descr_aes, &buff[sizeof(int)], sizeof(ca_descr_aes_t));
      ca_descr_aes.index = ntohl(ca_descr_aes.index);
      ca_descr_aes.parity = ntohl(ca_descr_aes.parity);
      decsa->SetAes(ca_descr_aes.index, true);
      decsa->SetDescrAes(&ca_descr_aes, false);
      DEBUGLOG("%s: Got CA_SET_DESCR_AES request, adapter_index=%d, index=%x", __FUNCTION__, adapter_index, ca_descr_aes.index);
    }
    else if (*request == CA_SET_DESCR_MODE)
    {
      memcpy(&ca_descr_mode, &buff[sizeof(int)], sizeof(ca_descr_mode_t));
      ca_descr_mode.index = ntohl(ca_descr_mode.index);
      ca_descr_mode.algo = (ca_descr_algo) ntohl(ca_descr_mode.algo);
      ca_descr_mode.cipher_mode = (ca_descr_cipher_mode) ntohl(ca_descr_mode.cipher_mode);
      decsa->SetAlgo(ca_descr_mode.index, ca_descr_mode.algo);
      decsa->SetCipherMode(ca_descr_mode.index, ca_descr_mode.cipher_mode);
      DEBUGLOG("%s: Got CA_SET_DESCR_MODE request, adapter_index=%d, index=%x", __FUNCTION__, adapter_index, ca_descr_mode.index);
    }
    else if (*request == CA_SET_DESCR_DATA)
    {
      memcpy(&ca_descr_data, &buff[sizeof(int)], sizeof(ca_descr_data_t));
      ca_descr_data.index = ntohl(ca_descr_data.index);
      ca_descr_data.parity = (ca_descr_parity) ntohl(ca_descr_data.parity);
      ca_descr_data.data_type = (ca_descr_data_type) ntohl(ca_descr_data.data_type);
      ca_descr_data.length = ntohl(ca_descr_data.length);
      memcpy(&data, &buff[sizeof(int) + 16], 16 + ca_descr_data.length - sizeof(ca_descr_data_t));

      //We have a size of CA_SET_DESCR_DATA message > size of ca_descr_data_t because the ca_descr_data.data is a pointer
      //Size of CA_SET_DESCR_DATA message is a 16 + ca_descr_data.length, see Oscam module-dvbapi.c
      rc = Read(data + (16 + ca_descr_data.length - sizeof(ca_descr_data_t)), ca_descr_data.length - (16 + ca_descr_data.length - sizeof(ca_descr_data_t)));

      if(rc)
      {
        DEBUGLOG("%s: Got CA_SET_DESCR_DATA request, adapter_index=%d, index=%x", __FUNCTION__, adapter_index, ca_descr_data.index);
        ca_descr_data.data = data;
        decsa->SetAes(ca_descr_aes.index, false);
        decsa->SetData(&ca_descr_data, false);
      }
    }
    else if (*request == DMX_SET_FILTER)
    {
      unsigned char demux_index = buff[4];
      unsigned char filter_num = buff[5];
      int i = 6;

      uint16_t *pid_ptr = (uint16_t *) &buff[i];
      sFP2.pid = ntohs(*pid_ptr);
      i += 2;

      memcpy(&sFP2.filter.filter, &buff[i], 16);
      i += 16;
      memcpy(&sFP2.filter.mask, &buff[i], 16);
      i += 16;
      memcpy(&sFP2.filter.mode, &buff[i], 16);
      i += 16;

      uint32_t *timeout_ptr = (uint32_t *) &buff[i];
      sFP2.timeout = ntohl(*timeout_ptr);
      i += 4;

      uint32_t *flags_ptr = (uint32_t *) &buff[i];
      sFP2.flags = ntohl(*flags_ptr);

      DEBUGLOG("%s: Got DMX_SET_FILTER request, adapter_index=%d, pid=%X, demux_idx=%d, filter_num=%d", __FUNCTION__, adapter_index, sFP2.pid, demux_index, filter_num);
      filter->SetFilter(adapter_index, sFP2.pid, 1, demux_index, filter_num, sFP2.filter.filter, sFP2.filter.mask);
    }
    else if (*request == DMX_STOP)
    {
      unsigned char demux_index = buff[4];
      unsigned char filter_num = buff[5];

      uint16_t *pid_ptr = (uint16_t *) &buff[6];
      uint16_t pid = ntohs(*pid_ptr);

      DEBUGLOG("%s: Got DMX_STOP request, adapter_index=%d, pid=%X, demux_idx=%d, filter_num=%d", __FUNCTION__, adapter_index, pid, demux_index, filter_num);
      filter->SetFilter(adapter_index, pid, 0, demux_index, filter_num, NULL, NULL);
    }
    else if (*request == DVBAPI_SERVER_INFO)
    {
      uint16_t *proto_ver_ptr = (uint16_t *) &buff[4];
      protocol_version = ntohs(*proto_ver_ptr);
      DEBUGLOG("%s: Got SERVER_INFO: %s, protocol_version = %d", __FUNCTION__, &buff[6], protocol_version);
    }
    else if (*request == DVBAPI_ECM_INFO)
    {
      char cardsystem[255];
      char reader[255];
      char from[255];
      char protocol[255];
      unsigned char len, hops;
      int i = 4;

      uint16_t *sid_ptr = (uint16_t *) &buff[i];       //ServiceID
      uint16_t sid = ntohs(*sid_ptr);
      i += 2;

      uint16_t *caid_ptr = (uint16_t *) &buff[i];      //CAID
      uint16_t caid = ntohs(*caid_ptr);
      i += 2;

      uint16_t *pid_ptr = (uint16_t *) &buff[i];       //PID
      uint16_t pid = ntohs(*pid_ptr);
      i += 2;

      uint32_t *prid_ptr = (uint32_t *) &buff[i];      //ProviderID
      uint32_t prid = ntohl(*prid_ptr);
      i += 4;

      uint32_t *ecmtime_ptr = (uint32_t *) &buff[i];   //ECM time
      uint32_t ecmtime = ntohl(*ecmtime_ptr);

      //cardsystem name
      rc &= Read(&len, 1);                             //string length
      rc &= Read((unsigned char*)cardsystem, len);
      cardsystem[len] = 0;                             //terminate the string

      //reader name
      rc &= Read(&len, 1);                             //string length
      rc &= Read((unsigned char*)reader, len);
      reader[len] = 0;                                 //terminate the string

      //source (from)
      rc &= Read(&len, 1);                             //string length
      rc &= Read((unsigned char*)from, len);
      from[len] = 0;                                   //terminate the string

      //protocol name
      rc &= Read(&len, 1);                             //string length
      rc &= Read((unsigned char*)protocol, len);
      protocol[len] = 0;                               //terminate the string

      rc &= Read(&hops, 1);                            //hops

      DEBUGLOG("%s: Got ECM_INFO: adapter_index=%d, SID = %04X, CAID = %04X (%s), PID = %04X, ProvID = %06X, ECM time = %d ms, reader = %s, from = %s, protocol = %s, hops = %d", __FUNCTION__, adapter_index, sid, caid, cardsystem, pid, prid, ecmtime, reader, from, protocol, hops);
      if(rc)
        capmt->UpdateEcmInfo(adapter_index, sid, caid, pid, prid, ecmtime, cardsystem, reader, from, protocol, hops);
    }
    else
      DEBUGLOG("%s: Unknown request: %02X %02X %02X %02X", __FUNCTION__, request[0], request[1], request[2], request[3]);

    if (!rc)
    {
      DEBUGLOG("%s: Error reading data - resetting connection", __FUNCTION__);
      CloseConnection();
    }
  }
}

bool SocketHandler::pollfd(int timeout_ms, bool in) {
    struct pollfd p;
    p.fd = sock;
    p.events = in ? POLLIN : POLLOUT;
    p.revents = 0;

    return (::poll(&p, 1, timeout_ms) > 0);
}

bool SocketHandler::WriteData(unsigned char* data, int len, int timeout_ms) {
    int written = 0;

    while (written < len)
    {
      if (pollfd(timeout_ms, false) == 0)
        return false;

      int rc = send(sock, (const void*)(data + written), len - written, MSG_DONTWAIT | MSG_NOSIGNAL);

      if (rc == -1 || rc == 0)
      {
        if (errno == EAGAIN)
          continue;

        ERRORLOG("%s: error writing to socket - %i (%s)", __FUNCTION__, errno, strerror(errno));
        return false;
      }

      DEBUGLOG("socket_fd=%d len=%d wrote=%d", sock, len, rc);
      written += rc;
    }

    return true;
}

int SocketHandler::ReadData(unsigned char* data, int len, int timeout_ms) {
    int read = 0;

    while(read < len)
    {
      if (pollfd(timeout_ms, true) == 0)
        return ETIMEDOUT;

      int rc = recv(sock, (char*)(data + read), len - read, MSG_DONTWAIT);

      if (rc == 0)
        return ECONNRESET;

      if (rc == -1)
      {
        if (errno == EAGAIN)
          continue;

        if (errno == EBADF || errno == ENOTCONN)
          return ECONNRESET;

        ERRORLOG("%s: error reading from socket - %i (%s)", __FUNCTION__, errno, strerror(errno));
        return errno;
      }

      read += rc;
    }

    return 0;
}
