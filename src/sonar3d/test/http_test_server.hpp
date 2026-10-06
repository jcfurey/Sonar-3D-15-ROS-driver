// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace sonar3d::testing
{

// Minimal HTTP/1.1 server answering one request per connection.
class HttpServer
{
public:
  struct Request
  {
    std::string method;
    std::string path;
    std::string body;
  };
  struct Response
  {
    int status{204};
    std::string body;
    std::chrono::milliseconds delay{0};
  };

  HttpServer()
  : socket_(socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0))
  {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t size = sizeof(address);
    if (socket_ < 0 || bind(socket_, reinterpret_cast<sockaddr *>(&address), size) < 0 ||
      listen(socket_, 8) < 0 ||
      getsockname(socket_, reinterpret_cast<sockaddr *>(&address), &size) < 0)
    {
      throw std::runtime_error("could not start test HTTP server");
    }
    port_ = ntohs(address.sin_port);
    thread_ = std::thread([this] {serve();});
  }

  ~HttpServer()
  {
    stopping_ = true;
    shutdown(socket_, SHUT_RDWR);
    close(socket_);
    thread_.join();
  }

  void respond(const std::string & route, Response response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    responses_[route] = std::move(response);
  }

  std::vector<Request> requests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

  std::uint16_t port() const {return port_;}

private:
  void serve()
  {
    while (!stopping_) {
      const int connection = accept(socket_, nullptr, nullptr);
      if (connection < 0) {
        continue;
      }
      std::string data;
      char buffer[4096];
      std::size_t header_end = std::string::npos;
      std::size_t length = 0;
      while (true) {
        const auto received = recv(connection, buffer, sizeof(buffer), 0);
        if (received <= 0) {
          break;
        }
        data.append(buffer, static_cast<std::size_t>(received));
        if (header_end == std::string::npos) {
          header_end = data.find("\r\n\r\n");
          if (header_end != std::string::npos) {
            const auto field = data.find("Content-Length: ");
            if (field != std::string::npos && field < header_end) {
              length = std::stoul(data.substr(field + 16));
            }
          }
        }
        if (header_end != std::string::npos && data.size() >= header_end + 4 + length) {
          break;
        }
      }
      Request request;
      const auto space = data.find(' ');
      request.method = data.substr(0, space);
      request.path = data.substr(space + 1, data.find(' ', space + 1) - space - 1);
      if (header_end != std::string::npos) {
        request.body = data.substr(header_end + 4, length);
      }
      Response response{404, "", std::chrono::milliseconds(0)};
      {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.push_back(request);
        if (const auto found = responses_.find(request.method + " " + request.path);
          found != responses_.end())
        {
          response = found->second;
        }
      }
      std::this_thread::sleep_for(response.delay);
      const auto reply = "HTTP/1.1 " + std::to_string(response.status) + " X\r\nContent-Length: " +
        std::to_string(response.body.size()) + "\r\nConnection: close\r\n\r\n" + response.body;
      static_cast<void>(send(connection, reply.data(), reply.size(), MSG_NOSIGNAL));
      close(connection);
    }
  }

  int socket_;
  std::uint16_t port_{};
  std::atomic<bool> stopping_{false};
  std::mutex mutex_;
  std::map<std::string, Response> responses_;
  std::vector<Request> requests_;
  std::thread thread_;
};

}  // namespace sonar3d::testing
