#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include "TCPConnection.h"
#include <list>

using boost::asio::ip::tcp;

#ifndef TCPSERVER_H
#define	TCPSERVER_H

class TCPServer {
public:

public:
	TCPServer(boost::asio::io_service& io_service, std::size_t port);
	virtual ~TCPServer();
        
	void broadcast(std::string message);

private:
	std::list<TCPConnection::pointer> connections;
	tcp::acceptor acceptor_;

	void start_accept();
	void handle_accept(TCPConnection::pointer new_connection, const boost::system::error_code& error);
};

#endif	/* TCPSERVER_H */

