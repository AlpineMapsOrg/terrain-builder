#pragma once

class ITrackable
{
    virtual bool has_changed() = 0;
    virtual void reset_changed() = 0;
};
