#include <kilolib.h>

// Define the number of neighbors we accept
#define maxN 4

// Declare a structure to represent each neighbor
typedef struct {
    int id;
    int distance;
    uint8_t timestamp;
} neighbor_t;

// Message flags
int new_message = 0;
int message_sent = 0;

// Message to transmit
message_t message;

// Create list of neighbors
neighbor_t neighbors[maxN];

// Track the number of nearest neighbors
int N_neighbors = 0;

void setup() {
    // Initialize message:
    // The type is always NORMAL.
    message.type = NORMAL;
    // Seed the software random number generator using
    // a hardware random number generator
    rand_seed(rand_hard());
    // Give the kilobot a unique ID
    kilo_uid = rand_soft();
    
    // Some dummy data as an example.
    message.data[0] = kilo_uid;
    // It's important that the CRC is computed after the data has been set;
    // otherwise it would be wrong and the message would be dropped by the
    // receiver.
    message.crc = message_crc(&message);
}

// Removes neighbors who are no longer in contact
void purge(void){
    int8_t i;
    
    for (i = N_neighbors-1; i >= 0; i--){
        
        if (kilo_ticks - neighbors[i].timestamp  > 64) //32 ticks = 1 s
        {
            //this one is too old.
            neighbors[i] = neighbors[N_neighbors-1];
            //replace it by the last entry
            N_neighbors--;
        }
    }
}

void loop() {
    if (new_message==0){
    purge();
    if (N_neighbors == 0)
        set_color(RGB(1, 0, 0));
    else if (N_neighbors == 1)
        set_color(RGB(0, 0, 1));
    
    else if (N_neighbors == 2)
        set_color(RGB(0, 1, 0));
    }
}

message_t *message_tx()
{
    return &message;
}

void message_tx_success()
{
    // Set the flag on message transmission.
    message_sent = 1;
}



void message_rx(message_t *message, distance_measurement_t *distance)
{
    int i = 0;
    // Set the flag on message reception.
    new_message = 1;
    // MY IDS ARE NOT UNIQUE
    int id = message->data[0];
    // Check to see if the id is in the list of neighbors
    for (i = 0; i < N_neighbors; i++)
        if (neighbors[i].id == id)
        {// found it
            neighbors[i].distance = estimate_distance(distance);
            neighbors[i].timestamp = kilo_ticks;
            break;
        }
    //if (N_neighbors < maxN-1) // if we have too many neighbors,
    if (i == N_neighbors){
        N_neighbors++;
        // i now points to where this message should be stored
        neighbors[i].id = id;
        neighbors[i].distance = estimate_distance(distance);
        neighbors[i].timestamp = kilo_ticks;
    }
    new_message = 0;
    
}



int main() {
    // initialize hardware
    kilo_init();
    // Register the message_tx callback function
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;
    // Register the message_rx callback function.
    kilo_message_rx = message_rx;
    // start program
    kilo_start(setup, loop);
    
    return 0;
}



