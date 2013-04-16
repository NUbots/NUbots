#include "TCPServer.h"

TCPServer::TCPServer(boost::asio::io_service& io_service, std::size_t port) : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)) {
	start_accept();
}

TCPServer::~TCPServer() {
}

void TCPServer::broadcast(std::string message) {
	for (std::list<TCPConnection::pointer>::iterator connection = connections.begin(); connection != connections.end(); ++connection) {
		(*connection)->send(message);
	}
}

void TCPServer::start_accept() {
	TCPConnection::pointer new_connection = TCPConnection::create(acceptor_.io_service());
	acceptor_.async_accept(new_connection->socket(), boost::bind(&TCPServer::handle_accept, this, new_connection, boost::asio::placeholders::error));
}

void TCPServer::handle_accept(TCPConnection::pointer new_connection, const boost::system::error_code& error) {
	if (!error) {
		new_connection->start();
		connections.push_back(new_connection);
		start_accept();
	}
}
