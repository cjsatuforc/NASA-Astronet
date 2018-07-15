function ptcallback(src,evt)
    global ptcloud
    ptcloud.PreserveStructureOnRead = evt.PreserveStructureOnRead;
    ptcloud.Data = evt.Data; 
    ptcloud.Height = evt.Height;
    ptcloud.Width = evt.Width;
    ptcloud.IsBigendian = evt.IsBigendian;
    ptcloud.PointStep = evt.PointStep;
    ptcloud.RowStep = evt.RowStep;
    ptcloud.IsDense = evt.IsDense;
    ptcloud.Fields = evt.Fields;
end