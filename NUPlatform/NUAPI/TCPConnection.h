#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

#ifndef TCPCONNECTION_H
#define	TCPCONNECTION_H

class TCPConnection : public boost::enable_shared_from_this<TCPConnection> {
public:
	typedef boost::shared_ptr<TCPConnection> pointer;

	static pointer create(boost::asio::io_service& io_service);
	tcp::socket& socket();
	void start();
	void send(std::string data);

	virtual ~TCPConnection();

private:
	tcp::socket socket_;

	TCPConnection(boost::asio::io_service& io_service);
	void handle_write(const boost::system::error_code&, size_t);
        bool done;
};

#endif	/* TCPCONNECTION_H */

