#include <algorithm>
#include <iostream>

#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

#include "gen/GazeboObjectState.hpp"

int subscriber_main(int domain_id, int sample_count)
{
    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(domain_id);

    // Create a Topic -- and automatically register the type
    dds::topic::Topic<GazeboState::DistanceObjectState> topic(participant, "DistanceObjectGazebo");

    // Create a DataReader with default Qos (Subscriber created in-line)
    dds::sub::DataReader<GazeboState::DistanceObjectState> reader(dds::sub::Subscriber(participant), topic);

    // Create a ReadCondition for any data on this reader and associate a handler
    int count = 0;
    dds::sub::cond::ReadCondition read_condition(
        reader,
        dds::sub::status::DataState::any(),
        [&reader, &count]()
    {
        // Take all samples
        dds::sub::LoanedSamples<GazeboState::DistanceObjectState> samples = reader.take();
        for (auto sample : samples){
            if (sample.info().valid()){
                count++;
                std::cout << sample.data() << std::endl; 
            }   
        }

    } // The LoanedSamples destructor returns the loan
    );

    // Create a WaitSet and attach the ReadCondition
    dds::core::cond::WaitSet waitset;
    waitset += read_condition;

    while (count < sample_count || sample_count == 0) {
        // Dispatch will call the handlers associated to the WaitSet conditions
        // when they activate
        waitset.dispatch(dds::core::Duration(0.1)); // Wait up to 4s each time
    }
    return 1;
}

int main(int argc, char *argv[])
{

    int domain_id = 0;
    int sample_count = 0; // infinite loop

    if (argc >= 2) {
        domain_id = atoi(argv[1]);
    }
    if (argc >= 3) {
        sample_count = atoi(argv[2]);
    }

    // To turn on additional logging, include <rti/config/Logger.hpp> and
    // uncomment the following line:
    // rti::config::Logger::instance().verbosity(rti::config::Verbosity::STATUS_ALL);

    try {
        subscriber_main(domain_id, sample_count);
    } catch (const std::exception& ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in subscriber_main(): " << ex.what() << std::endl;
        return -1;
    }

    // RTI Connext provides a finalize_participant_factory() method
    // if you want to release memory used by the participant factory singleton.
    // Uncomment the following line to release the singleton:
    //
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return 0;
}
