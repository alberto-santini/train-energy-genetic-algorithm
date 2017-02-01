#include <assert.h>
#include <stdio.h>
#include "instance.h"
#include "lookup.h"

int main() {
    Instance inst = read_instance("../data/test.json");
    Lookup l = generate_lookup_tables(&inst);

    // print_instance(&i);
    print_lookup_tables(&l, &inst);

    free_lookup_tables(&l);
    free_instance(&inst);

    return 0;
}