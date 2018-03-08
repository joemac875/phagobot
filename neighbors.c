#include <kilolib.h>
#include <stdbool.h>
#define DEBUG
#include <debug.h>
// Only needed for debugging
#include <inttypes.h>

// Define the number of neighbors we accept
#define maxN 4

// Declare a structure to represent each neighbor
typedef struct {
    int id;
    int distance;
    int gradient;
    uint32_t timestamp;
} neighbor_t;

// Message flags
int new_message = 0;
int message_sent = 0;

// dummy values for light reading functionality
int max_reading = 0;
bool mine = true;
int own_gradient = 0;
int received_gradient = 0;
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
    // Instead of Grabbing a light reading we'll just use the id as a dummy 'id'
    max_reading = kilo_uid;
    
    // Load data
    message.data[0] = kilo_uid;
    message.data[1] = max_reading;
    message.data[2] = own_gradient;
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
    /*
    if (N_neighbors == 0)
        set_color(RGB(1, 0, 0));
    else if (N_neighbors == 1)
        set_color(RGB(0, 0, 1));
    
    else if (N_neighbors == 2)
        set_color(RGB(0, 1, 0));
     */
        if (mine){
            own_gradient = 0;
            set_color(RGB(1, 1, 1)); // White
        }
        else if (!mine){
            int lowest_grad = 255;
            for (int i = 0; i < N_neighbors; i++){
                if (neighbors[i].gradient < lowest_grad)
                {// found it
                    lowest_grad = neighbors[i].gradient;
                }
            }
            own_gradient = lowest_grad + 1;
            if (own_gradient == 1)
            {
                set_color(RGB(1, 0, 0)); // Red
            }
            else if (own_gradient == 2)
            {
                set_color(RGB(0, 1, 0)); // Green
            }
            else if (own_gradient == 3)
            {
                set_color(RGB(0, 0, 1)); // Blue
            }
            else if (own_gradient == 4)
            {
                set_color(RGB(1, 0, 1)); // Magenta
            }
            else if (own_gradient >= 5)
            {
                set_color(RGB(1, 1, 0)); // Yellow
            }
        }
        
        
    }
}

message_t *message_tx()
{
    // Change the Message Data
    message.data[1] = max_reading;
    message.data[2] = own_gradient;
    // Recompute the checksum
    message.crc = message_crc(&message);
    return &message;
    
}

void message_tx_success()
{
    printf("id %i max %i \n", kilo_uid, max_reading);
    // Set the flag on message transmission.
    message_sent = 1;
    // Forget old neigbors
    purge();
}



void message_rx(message_t *message, distance_measurement_t *distance)
{
    int i = 0;
    // Set the flag on message reception.
    new_message = 1;
    // MY IDS ARE NOT UNIQUE
    int id = message->data[0];
    // Check to see if the id is in the list of neighbors
    for (i = 0; i < N_neighbors; i++){
        //printf("Neighbor Count %i id %i time %" PRId32 "\n",N_neighbors, neighbors[i].id,neighbors[i].timestamp);
        
        if (neighbors[i].id == id)
        {// found it
            neighbors[i].distance = estimate_distance(distance);
            neighbors[i].timestamp = kilo_ticks;
            neighbors[i].gradient = message->data[2];
            break;
        }
    }
    //if (N_neighbors < maxN-1) // if we have too many neighbors,
    if (i == N_neighbors){
        N_neighbors++;
        // i now points to where this message should be stored
        neighbors[i].id = id;
        neighbors[i].distance = estimate_distance(distance);
        neighbors[i].timestamp = kilo_ticks;
        neighbors[i].gradient = message->data[2];
    }
    new_message = 0;
    
    
    // See if their light reading is higher
    if (message->data[1] > max_reading){
        max_reading = message->data[1];
        mine = false;
    }
    
}



int main() {
    debug_init();
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



