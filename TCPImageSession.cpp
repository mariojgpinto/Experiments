/** 
 * @file	TCPImageSession.cpp 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	April, 2014
 * @brief	Implementation of the TCPImageSession class.
 */
#include "TCPServer.h"

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
TCPImageSession::TCPImageSession(TCPServer* server, boost::asio::io_service* io_service){
	this->_server = server;
	
	this->_connected = false;
	this->_running = false;
	
	this->_write_deque = new std::deque<cv::Mat>();

	if(io_service){
		this->_socket = new boost::asio::ip::tcp::socket(*io_service);
	}
}

/**
 * @brief	.
 * @details	.
 */
TCPImageSession::~TCPImageSession(){
	this->stop_session();
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::stop_session(){
	this->_running = false;
	this->_connected = false;

	Sleep(50);
	
	if(this->_socket){
		try{
			this->_socket->close();
		}
		catch(boost::exception& e){
			printf("Exception at TCPImageSession: TCPImageSession::stop_client\n");
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
void TCPImageSession::start(){
	this->_connected = true;

	this->_write_thread = new boost::thread(&TCPImageSession::run_write_thread,this);
	this->_read_thread = new boost::thread(&TCPImageSession::run_read_thread,this);
}

//-----------------------------------------------------------------------------
// RUN
//-----------------------------------------------------------------------------
/**
 * @brief	.
 * @details	.
 */
void TCPImageSession::run_write_thread(){
	this->_running = true;

	while(this->_running){
		if(!this->_connected){
			this->_running = false;
		}
		else{
			while(this->_write_deque->size()){
				try{
					int channels = this->_write_deque->at(0).channels();
					size_t request_length = this->_write_deque->at(0).rows * 
											this->_write_deque->at(0).cols *
											this->_write_deque->at(0).channels();

					boost::asio::write(*this->_socket, 
										boost::asio::buffer(this->_write_deque->at(0).data, request_length));
				}
				catch(boost::exception& e){
					printf("TCPImageSession, Exception at: \"run_write_thread\"\n");
					if(this->_connected){
						this->stop_session();
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

void TCPImageSession::run_read_thread(){
	this->_running = true;

	while(this->_running){
		if(!this->_connected){
			this->_running = false;
		}
			
		try{
			size_t bytes_transferred = this->_socket->read_some(boost::asio::buffer(this->_read_buffer, TCPImageSession::_buff_size));
		
			if(bytes_transferred){
				//this->_read_deque->push_back(new std::string(this->_read_buffer,bytes_transferred));
			}
		}
		catch(...){
			printf("TCPImageSession, Error on \"run_read_thread()\".\n");
			if(this->_connected){
				this->stop_session();
			}
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
void TCPImageSession::add_image(cv::Mat image){
	this->_write_mutex.lock();
		this->_write_deque->push_back(image.clone());
	this->_write_mutex.unlock();
	
	this->_condition.notify_all();
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------

/**
 * @brief	.
 * @details	.
 */
bool TCPImageSession::is_alive(){
	return this->_connected && this->_running;
}

/**
 * @brief	.
 * @details	.
 */
boost::asio::ip::tcp::socket* TCPImageSession::get_socket(){
	return _socket;
}