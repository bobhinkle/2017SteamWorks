package SubSystems;

/**
 * @author Jared Russell
 */
public interface Controller extends Runnable
{
    public void reset();
    public boolean onTarget();
    public void loadProperties();
}
