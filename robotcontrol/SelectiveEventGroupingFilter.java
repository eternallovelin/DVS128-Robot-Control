package jaer.myjaer.robotcontrol;

import jaer.myjaer.util.EventGroup;
import jaer.myjaer.util.ModifiedXMLFormatter;
import jaer.myjaer.util.Pixel;
import jaer.myjaer.util.UtilClass;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Float;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;

import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.PolarityEvent;
import net.sf.jaer.event.PolarityEvent.Polarity;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;

public class SelectiveEventGroupingFilter extends EventFilter2D implements FrameAnnotater{

	private Hashtable<Point,Pixel> pix_map;
	private LinkedList<EventGroup> event_groups;
	
	private static final int PIX_NEIGHBORHOOD = 1;
	// If we see 4 neighboring events with the same polarity, group them
	private static final int NUM_EVENT_THRESHOLD = 4;
	
	private boolean draw = false;
	
	private int num_groups = 0;
	
	private boolean annotate = getPrefs().getBoolean("SelectiveEventGroupingFilter.annotate", false);
	
	private static Logger logger;
	private static FileHandler file_handler;
	private static Level log_level = Level.ALL;
	private static ModifiedXMLFormatter formatter;
	private static String xsl_file = "";
	
	public SelectiveEventGroupingFilter(AEChip chip) {
		super(chip);
		initFilter();
	}

	@Override
	public void annotate(GLAutoDrawable drawable) {
			if(draw && annotate)
			{
				LinkedList<EventGroup> copy=  (LinkedList<EventGroup>)event_groups.clone();
				GL gl=drawable.getGL();
				for(EventGroup group : copy)
				{
					float weight = 1;
					gl.glBegin(GL.GL_QUADS);
					Point2D.Float location = group.get_group_location();
					gl.glVertex2d(location.x - weight/**group.get_group_size()*/, location.y - weight/**group.get_group_size()*/);
					gl.glVertex2d(location.x - weight/**group.get_group_size()*/, location.y + weight/**group.get_group_size()*/);
					gl.glVertex2d(location.x + weight/**group.get_group_size()*/, location.y + weight/**group.get_group_size()*/);
					gl.glVertex2d(location.x + weight/**group.get_group_size()*/, location.y - weight/**group.get_group_size()*/);
					gl.glEnd();
				}
			}
	}

	@Override
	public EventPacket<?> filterPacket(EventPacket<?> in) {
		draw = false;
		if(pix_map.size() == 0)
			populate_pix_map();
		else
			clear();
		
		for(Object obj:in)
		{
			PolarityEvent p_event = (PolarityEvent)obj;
			Polarity pol = p_event.polarity;
			Point event_location = new Point(p_event.x,p_event.y);
			
			Pixel pix = pix_map.get(event_location);
			int flag = pix.assign_event(p_event);
			if(flag == 1)	// Apparently, two events may have the same x-y coordinates and this messes up grouping!
			{
				ArrayList<Pixel> neighbors = pix.get_pixels_by_ev_polarity(pol);
				
				for (Pixel neighbor: neighbors)
				{
					if(neighbor.belongs_to_group())
					{
						EventGroup neig_gr = neighbor.get_group_id();
						if(!pix.belongs_to_group())
						{
							EventGroup eg = neighbor.get_group_id();
							eg.add_event_to_group(pix);						
						}
						else if(pix.get_group_id() != neighbor.get_group_id())
						{
							EventGroup pix_gr_id = pix.get_group_id();
							merge(neig_gr,pix_gr_id);
						}
					}
				}
				if(!pix.belongs_to_group())
				{
					if(neighbors.size() >= NUM_EVENT_THRESHOLD)
					{
						EventGroup e_group = new EventGroup(p_event, pol);
						event_groups.add(e_group);
						pix.set_group_id(e_group);
						for(Pixel neig : neighbors)
						{
							e_group.add_event_to_group(neig);
							e_group.add_event_to_group(neig.get_pixels_by_ev_polarity(pol));
						}
					}
					else
					{
						for(Pixel neig: neighbors)
						{
							if(neig.get_pixels_by_ev_polarity(pol).size() >= NUM_EVENT_THRESHOLD)
							{
								EventGroup e_group = new EventGroup(p_event,pol);
								event_groups.add(e_group);
								pix.set_group_id(e_group);
								for(Pixel neig_neig: neig.get_pixels_by_ev_polarity(pol))
								{
									e_group.add_event_to_group(neig_neig);
									e_group.add_event_to_group(neig_neig.get_neighbours());
								}
								break;
							}
						}
					}
				}
			}
					
		}
		num_groups = event_groups.size();
		draw = true;
		if(num_groups > 0)
		{
			Object[] dat = UtilClass.init_log_dat(logger,2, "GROUPS", new Object[]{event_groups});
			logger.log(Level.FINEST, " ", dat);
		}
		
			
		return in;
		
	}

	public void clear() {
		Enumeration<Pixel> element_enumerator = pix_map.elements();
		while(element_enumerator.hasMoreElements())
			element_enumerator.nextElement().clear();
		event_groups = new LinkedList<EventGroup>();
				
	}

	public void populate_pix_map() {
		for(int x=0;x<=chip.getSizeX();x++)
		{
			for(int y=0;y<=chip.getSizeY();y++)
			{
				Point pix_location = new Point(x,y);
				if(pix_map.containsKey(pix_location))
					add_pixel_neighbours(pix_map.get(pix_location));
				else
				{
					Pixel pix = new Pixel(pix_location);
					pix_map.put(pix_location, pix);
					add_pixel_neighbours(pix);
				}
			}
		}		
	}
	
	/** Populate the pixel neighbourhood of a given pixel
	 * @param pix the pixel
	 */
	private void add_pixel_neighbours(Pixel pix)
	{	
		Point pix_location = pix.get_location();
		
		for(int pix_x = pix_location.x - PIX_NEIGHBORHOOD; pix_x<= pix_location.x + PIX_NEIGHBORHOOD; pix_x++)
		{
			if(pix_x < 0 || pix_x > 128)
				continue;
			for(int pix_y = pix_location.y - PIX_NEIGHBORHOOD; pix_y <= pix_location.y + PIX_NEIGHBORHOOD; pix_y++)
			{
				if(pix_y < 0 || pix_y > 128)
					continue;
				Point neighbour_loc = new Point(pix_x,pix_y);
				if(neighbour_loc.getLocation().equals(pix_location.getLocation()))
					continue;
				if(pix_map.containsKey(neighbour_loc))
				{
					pix.add_neighbour(pix_map.get(neighbour_loc));
				}
				else
				{
					Pixel neighbour_pix = new Pixel(neighbour_loc);
					pix_map.put(neighbour_loc, neighbour_pix);
					pix.add_neighbour(neighbour_pix);
				}
			}
		}
		
	}

	@Override
	public void resetFilter() {
		
		
	}

	@Override
	public void initFilter() {
		pix_map = new Hashtable<Point, Pixel>();
		event_groups = new LinkedList<EventGroup>();
	
		logger = Logger.getLogger(this.getClass().getName());
		try 
		{
			file_handler = new FileHandler("/home/vanxa/workspace/java/java/log/event_groups.xml", false);
		} 
		catch (Exception e) 
		{
			e.printStackTrace();
		}
			
		formatter = new ModifiedXMLFormatter(xsl_file);
		file_handler.setFormatter(formatter);
		logger.addHandler(file_handler);
		logger.setLevel(log_level);
		logger.setUseParentHandlers(false);
	}
	
	public void merge(EventGroup gr_1, EventGroup gr_2)
	{
		
		for(PolarityEvent p_event:gr_2.get_events())
		{
			Pixel pix = pix_map.get(new Point(p_event.x,p_event.y));
			gr_1.add_event_to_group(pix);
			pix.set_group_id(gr_1);
		}
		event_groups.remove(gr_2);
	}
	
	public void setAnnotate(boolean annotate)
	{
		this.annotate = annotate;
		getPropertyChangeSupport().firePropertyChange("annotate", this.annotate, annotate);
		getPrefs().putBoolean("SelectiveEventGroupingFilter.annotate", annotate);
	}
	
	public boolean getAnnotate()
	{
		return annotate;
	}
	
	public LinkedList<EventGroup> get_event_groups()
	{
		return event_groups;
	}
	
	public void set_num_groups(int size)
	{
		this.num_groups = size;
	}
	
	public int get_num_groups()
	{
		return num_groups;
	}

}
