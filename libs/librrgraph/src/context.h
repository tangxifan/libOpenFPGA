/* IMPORTANT:
 * The following preprocessing flags are added to 
 * avoid compilation error when this headers are included in more than 1 times 
 */
#ifndef CONTEXT_H
#define CONTEXT_H

/* A Context is collection of state relating to a particular part of VPR
 *
 * This is a base class who's only purpose is to disable copying of contexts.
 * This ensures that attempting to use a context by value (instead of by reference)
 * will result in a compilation error.
 *
 * No data or member functions should be defined in this class!
 */
struct Context {
    //Contexts are non-copyable
    Context() = default;
    Context(Context&) = delete;
    Context& operator=(Context&) = delete;
    virtual ~Context() = default;
};

#endif
