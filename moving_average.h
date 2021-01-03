#ifndef __MOVING_AVERAGE__
#define __MOVING_AVERAGE__

template <typename T, size_t N>
class Moving_Average
{
public:
    void operator()(T sample)
    {
        if (num_samples_ < N)
        {
            samples_[num_samples_++] = sample;
            total_ += sample;
        }
        else
        {
            T &oldest = samples_[num_samples_++ % N];
            total_ += sample - oldest;
            oldest = sample;
        }
    }

    operator double() const { return total_ / std::min(num_samples_, N); }

    T get()
    {
        return total_ / std::min(num_samples_, N);
    }

private:
    T samples_[N];
    size_t num_samples_{0};
    T total_{0};
};

#endif