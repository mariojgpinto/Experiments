/** 
 * @file	TCPMessageSession.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	April, 2014
 * @brief	Implementation of the TCPMessageSession class.
 */
#include "TCPServer.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
TCPMessageSession::TCPMessageSession(TCPServer* server, boost::asio::io_service* io_service){
	this->_server = server;
	
	this->_connected = false;
	this->_running = false;
	
	this->_read_deque = new std::deque<std::string*>();
	this->_write_deque = new std::deque<std::string*>();

	if(io_service){
		this->_socket = new boost::asio::ip::tcp::socket(*io_service);
	}
}

/**
 * @brief	.
 * @details	.
 */
TCPMessageSession::~TCPMessageSession(){
	this->stop_session();
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::set_output_message_deque(std::deque<std::string*>* deque){
	if(deque){
		this->_read_deque = deque;
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::stop_session(){
	if(this->_connected){
		this->_running = false;
		this->_connected = false;

		Sleep(50);
	
		if(this->_socket){
			try{
				this->_socket->close();
			}
			catch(boost::exception& e){
				printf("Exception at TCPMessageSession: TCPMessageSession::stop_client\n");
			}
		}
	}
}

//-----------------------------------------------------------------------------
// INIT
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::start(){
	this->_connected = true;

	boost::asio::socket_base::receive_buffer_size option(TCPMessageSession::_buff_size);
	this->_socket->set_option(option);

	//this->_socket->async_read_some(	boost::asio::buffer(_buffer, TCPMessageSession::_buff_size),
	//								boost::bind(&TCPMessageSession::handle_read, this,
	//								boost::asio::placeholders::error,
	//								boost::asio::placeholders::bytes_transferred));

	this->_read_thread = new boost::thread(&TCPMessageSession::run_read_thread,this);
	this->_write_thread = new boost::thread(&TCPMessageSession::run_write_thread,this);
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::run_read_thread(){
	this->_running = true;

	while(this->_running){
		if(!this->_connected){
			this->_running = false;
			break;
		}
			
		try{
			size_t bytes_transferred = this->_socket->read_some(boost::asio::buffer(this->_read_buffer, TCPMessageSession::_buff_size));
		
			if(bytes_transferred){
				this->_read_deque->push_back(new std::string(this->_read_buffer,bytes_transferred));
			}
		}
		catch(...){
			printf("TCPMessageSession, Error on \"run_read_thread()\".\n");
			if(this->_connected){
				this->stop_session();
			}
		}
	}
}

/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::run_write_thread(){
	this->_running = true;

	while(this->_running){
		if(!this->_connected){
			this->_running = false;
		}
		else{
			while(this->_write_deque->size()){
				if(this->_write_deque->at(0)){
					try{
						size_t request_length = this->_write_deque->at(0)->length();
						boost::asio::write(*this->_socket, 
											boost::asio::buffer(this->_write_deque->at(0)->data(), 
																request_length));
					}
					catch(boost::exception& e){
						printf("TCPMessageSession, Exception at: \"run_write_thread\"\n");
						if(this->_connected){
							this->stop_session();
						}
					}
				}

				this->_write_mutex.lock();
					this->_write_deque->pop_front();
				this->_write_mutex.unlock();
			}
		}
		
		{
			boost::mutex::scoped_lock lock(this->_condition_mutex);
			this->_condition.wait(lock);
		}
	}
}

//-----------------------------------------------------------------------------
// MESSAGES
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPMessageSession::add_message(std::string msg){
	this->_write_mutex.lock();
		this->_write_deque->push_back(new std::string(msg));
	this->_write_mutex.unlock();
	
	this->_condition.notify_all();
}

/**
 * @brief	has_new_message().
 * @details has_new_message().
 */
int TCPMessageSession::has_new_message(){
	return this->_read_deque->size();
}

/**
 * @brief	consume_message(std::string& message).
 * @details consume_message(std::string& message).
 */
bool TCPMessageSession::consume_message(std::string& message){
	if(this->_read_deque->size()){
		this->_read_mutex.lock();
			message.assign(*this->_read_deque->at(0));
			this->_read_deque->pop_front();
		this->_read_mutex.unlock();

		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------

/**
 * @brief	.
 * @details	.
 */
bool TCPMessageSession::is_alive(){
	return this->_connected && this->_running;
}

/**
 * @brief	.
 * @details	.
 */
boost::asio::ip::tcp::socket* TCPMessageSession::get_socket(){
	return _socket;
}