SECTION_PROLOGUE(.vec,,)
{
    _vec_start = .;
    KEEP(*(.vec.*))
    _vec_end = .;
} GROUP_LINK_IN(VEC)
_vec_size = _vec_end - _vec_start;
