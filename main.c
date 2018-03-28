#include <kilolib.h>
#include <stdbool.h>
#define DEBUG
#include <debug.h>
// Only needed for debugging
#include <inttypes.h>
// Constants for orbit control.


// Constants for motion handling function.
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

int current_motion = STOP;
//int TOO_CLOSE_DISTANCE = 40;
//int DESIRED_DISTANCE = 60;
//int star_id = 0;

// Define the number of neighbors we accept
#define maxN 4

// Declare a structure to represent each neighbor
typedef struct {
    uint16_t id;
    int distance;
    int gradient;
    int reading;
    uint32_t timestamp;
} neighbor_t;

// Message flags
int new_message = 0;
int message_sent = 0;

// dummy values for light reading functionality
uint8_t my_reading;
uint8_t id_a, id_b;
uint16_t id_full;
bool mine = true;
int own_gradient = 0;
int received_gradient = 0;
// Message to transmit
message_t message;

// Create list of neighbors
neighbor_t neighbors[maxN];

// Track the number of nearest neighbors
int N_neighbors = 0;


// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an action if the motion is being changed.
    if (current_motion != new_motion)
    {
        current_motion = new_motion;
        
        if (current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == LEFT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

uint8_t max_reading(){
    int max = 0;
    for (int i = 0; i < N_neighbors; i++){
        if (neighbors[i].reading > max)
        {
            max = neighbors[i].reading;
        }
    }
    if (my_reading > max)
        return my_reading;
    return max;
}

void setup() {
    // Initialize message:
    // The type is always NORMAL.
    message.type = NORMAL;
    // Seed the software random number generator using
    // a hardware random number generator
    rand_seed(rand_hard());
    // Give the kilobot a unique ID
    id_a = rand_soft();
    id_b = rand_soft();
    my_reading = rand_soft();
    
    
    // Load data
    message.data[0] = id_a;
    message.data[1] = id_b;
    message.data[2] = my_reading;
    message.data[3] = own_gradient;
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
    if (message_sent == 1){
        if (max_reading() == my_reading){
            // My reading is the highest STATE 1
            own_gradient = 0;
            set_color(RGB(1, 1, 1)); // White
            set_motion(STOP);
        }
        else {
            // My light reading is not the highest STATE 2
            int lowest_grad = 255;
            for (int i = 0; i < N_neighbors; i++){
                if (neighbors[i].gradient < lowest_grad)
                {
                    lowest_grad = neighbors[i].gradient;
                }
            }
            own_gradient = lowest_grad + 1;
            printf("Gradient Value %i", own_gradient);
        }
        message_sent = 0;
    }
}



message_t *message_tx()
{
    int reading = max_reading();
    // Change the Message Data
    message.data[2] = reading;
    message.data[3] = own_gradient;
    // Recompute the checksum
    message.crc = message_crc(&message);
    return &message;
    
}

void message_tx_success()
{
    //printf("id %i max %i \n", kilo_uid, max_reading);
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
    uint16_t id = ((uint16_t)message->data[0] << 8) | message->data[1];
    // Check to see if the id is in the list of neighbors
    for (i = 0; i < N_neighbors; i++){
        //printf("Neighbor Count %i id %i time %" PRId32 "\n",N_neighbors, neighbors[i].id,neighbors[i].timestamp);
        
        if (neighbors[i].id == id)
        {// found it
            neighbors[i].distance = estimate_distance(distance);
            neighbors[i].timestamp = kilo_ticks;
            neighbors[i].reading = message->data[2];
            neighbors[i].gradient = message->data[3];
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
        neighbors[i].reading = message->data[2];
        neighbors[i].gradient = message->data[3];
    }
    new_message = 0;
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
