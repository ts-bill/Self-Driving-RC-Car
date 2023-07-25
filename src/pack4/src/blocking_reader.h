#include <string>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 
#include <boost/bind.hpp>

class blocking_reader
{
    boost::asio::serial_port& port;
    size_t timeout;
    char c;
    boost::asio::deadline_timer timer;
    bool read_error;

    void read_complete(const boost::system::error_code& error,size_t bytes_transferred)
    {
        read_error=(error||bytes_transferred==0);
        timer.cancel();
    }

    void time_out(const boost::system::error_code& error)
    {
        if(error)
        {
            return;
        }
        port.cancel();
    }

public:
    blocking_reader(boost::asio::serial_port& port,size_t timeout):port(port),timeout(timeout),timer(port.get_io_service()),read_error(true)
    {}
    bool read_char(char& val)
    {
        val=c='\0';
        port.get_io_service().reset();
        boost::asio::async_read(port,boost::asio::buffer(&c,1),boost::bind(&blocking_reader::read_complete,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
        timer.expires_from_now(boost::posix_time::milliseconds(timeout));
        timer.async_wait(boost::bind(&blocking_reader::time_out,this,boost::asio::placeholders::error));

        port.get_io_service().run();
        if(!read_error)
        {
            val=c;
        }
        return !read_error;
    }
    
};