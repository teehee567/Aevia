/* esp-hal normally permits only 8 KiB of remaining CPU0 RAM for the stack.
 * Acquisition, application protocol formatting and nested executor polls need
 * a larger reserve. Refuse future builds that silently consume this margin. */
ASSERT(_stack_start_cpu0 - _stack_end_cpu0 >= 32768,
       "Aevia requires at least 32 KiB CPU0 stack after static allocations.");
