/** 
 * @file	TCPClient.h 
 * @author	Mario Pinto (mario.pinto@ccg.pt) 
 * @date	February, 2014
 * @brief	Declaration of the TCPClient class.
 */
#ifndef _RDCC_TCP_CLIENT
#define _RDCC_TCP_CLIENT

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <deque>

class TCPClient{
	public:
		TCPClient(char* host = "localhost", char* port = "9991", bool threaded = true);
		~TCPClient();

		bool is_connected();

		void add_message(std::string msg);

		void stop_client();

	private:
		bool connect();

		void run();

	private:
		std::string							_host;
		std::string							_app_port;

		boost::asio::io_service*			_io_service;	/**< Connection's IO Service.*/

		bool								_flag_connected;
		boost::asio::ip::tcp::socket*		_socket;
		
		bool								_threaded;
		bool								_running;
		boost::thread*						_thread;
		boost::mutex						_mutex;			/**< Mutex for the contition _condition.*/
		boost::condition_variable			_condition;		/**< Condition variable to wait for new messages.*/

		std::deque<std::string*>			_app_messages_out;		/**< DeQueue of messages to be send.*/
};

#endif//_RDCC_TCP_CLIENT